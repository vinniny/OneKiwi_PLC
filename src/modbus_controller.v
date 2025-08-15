// modbus_controller.v (Verilog-2001 compatible)
// - Master/Slave handling
// - Parses RTU/ASCII frames (ASCII bytes passed through from bridge; hex decode inside)
// - Executes FC 01/02/03/04/05/06/0F/10

module modbus_controller #(
  parameter REG_WORDS = 64,
  parameter SCAN_MAX  = 16,
  // limit RX/TX buffer depth to keep synthesis time manageable
  // Increased to 256 to align buffer indexing widths
  parameter BUF_MAX   = 256
)(
  input  wire        clk,
  input  wire        rst,

  // Config
  input  wire        cfg_mode_master,
  input  wire        cfg_ascii_en,
  input  wire [15:0] cfg_map_base_qw_iw,

  // UART bridge byte stream + frame marks
  input  wire [7:0]  rx_b,
  input  wire        rx_b_v,
  input  wire        frame_start,
  input  wire        frame_end,

  output reg  [7:0]  tx_b,
  output reg         tx_b_v,
  input  wire        tx_b_rdy,

  // GPIO + DO write interface
  input  wire [31:0] di_status,
  output reg  [31:0] do_wdata,
  output reg  [31:0] do_wmask,
  output reg         do_we,

  // Status (wired to CSR)
  output reg         stat_crc_err,
  output reg         stat_lrc_err,
  output reg         stat_frame_to,

  // Scan engine CSRs (latched from CSR block)
  input  wire        scan_en,
  input  wire [3:0]  scan_retry_max,
  input  wire [15:0] scan_period_ms,
  input  wire [7:0]  scan_idx,
  input  wire [7:0]  scan_slave,
  input  wire [7:0]  scan_func,
  input  wire [15:0] scan_start_addr,
  input  wire [15:0] scan_qty,
  input  wire [15:0] scan_byte_count,
  input  wire [15:0] scan_wbase,
  input  wire [15:0] scan_rbase,
  output reg  [15:0] scan_cycles_done,
  output reg  [15:0] scan_err_count
);

  // ===== Internal process images =====
  reg [15:0] reg_holding [0:REG_WORDS-1];
  reg [15:0] reg_input   [0:REG_WORDS-1];

  // ===== RX buffer (per-frame) =====
  (* ram_style = "block" *) reg [7:0] rx_buf [0:BUF_MAX-1];
  reg [7:0] rx_len;
  reg       rx_in_ascii;

  // ===== TX buffer (per-frame) =====
  (* ram_style = "block" *) reg [7:0] tx_buf [0:BUF_MAX-1];
  reg [8:0] tx_len;
  reg [8:0] tx_idx;

  // ===== Shared temporaries =====
  integer i,j,k,bitn,ofs,base,tx_wr;
  integer bytes, blen, shamt;
  reg [7:0] b, nb0, nb1, bc, sum;
  // temp for ASCII hex -> binary
  (* ram_style = "block" *) reg [7:0] bin [0:BUF_MAX-1];
  reg [15:0] w, c, x;
  reg [31:0] new_do, msk;
  reg        ascii_err;
  reg [7:0]  tmp8;

  // CRC/LRC helpers (kept for compatibility; not used directly here)
  reg         crc_clr, lrc_clr;
  wire [15:0] crc_val;
  wire [7:0]  lrc_val;
  crc16_modbus u_crc(.clk(clk), .rst(rst), .clr(crc_clr), .data_i(rx_b), .valid_i(rx_b_v & frame_start), .crc_o(crc_val));
  lrc_ascii   u_lrc(.clk(clk), .rst(rst), .clr(lrc_clr), .data_i(rx_b), .valid_i(rx_b_v & frame_start), .lrc_o(lrc_val));

  // ===== Controller FSM =====
  localparam [3:0]
    S_IDLE    = 4'd0,
    S_COLLECT = 4'd1,
    S_PARSE   = 4'd2,
    S_EXEC    = 4'd3,
    S_BUILD   = 4'd4, // unused placeholder
    S_SEND    = 4'd5;

  reg [3:0]  st;
  reg [7:0]  dev_addr, func;
  reg [15:0] addr, qty, val;
  reg [7:0]  byte_count;

  // ----- helper functions -----
  function [15:0] crc16_sw;
    input [7:0] n;
    integer i0,j0; reg [15:0] c0; reg [7:0] d0;
    begin
      c0 = 16'hFFFF;
      for (i0=0;i0<BUF_MAX;i0=i0+1) begin
        if (i0 < n) begin
          d0 = rx_buf[i0[7:0]];
          for (j0=0;j0<8;j0=j0+1) begin
            // Extend byte to match 16-bit CRC width before XOR
            if (((c0 ^ {8'h00, d0}) & 16'h0001)!=16'h0000) c0=(c0>>1)^16'hA001; else c0=(c0>>1);
            d0 = d0 >> 1;
          end
        end
      end
      crc16_sw = c0;
    end
  endfunction

  function [7:0] hex2nib;
    input [7:0] c_in;
    begin
      // Default to 0xFF for invalid characters
      hex2nib = 8'hFF;
      if      (c_in >= "0" && c_in <= "9") hex2nib = c_in - "0";
      else if (c_in >= "A" && c_in <= "F") hex2nib = c_in - "A" + 8'd10;
      else if (c_in >= "a" && c_in <= "f") hex2nib = c_in - "a" + 8'd10;
    end
  endfunction

  // ===== Main sequential block =====
  always @(posedge clk) begin
    if (rst) begin
      st <= S_IDLE;
      rx_len <= 8'd0;
      tx_idx <= 9'd0;
      tx_len <= 9'd0;
      tx_b_v <= 1'b0;

      do_we    <= 1'b0;
      do_wmask <= 32'd0;
      do_wdata <= 32'd0;

      stat_crc_err  <= 1'b0;
      stat_lrc_err  <= 1'b0;
      stat_frame_to <= 1'b0;

      rx_in_ascii <= 1'b0;
    end else begin
      tx_b_v <= 1'b0;
      do_we  <= 1'b0;

      case (st)
        // =========================
        S_IDLE: begin
          rx_len <= 8'd0;
          if (frame_start) begin
            rx_in_ascii <= cfg_ascii_en;
            st <= S_COLLECT;
          end
        end

        // =========================
        S_COLLECT: begin
          if (rx_b_v) begin
            rx_buf[rx_len] <= rx_b;
              if ({1'b0, rx_len} != BUF_MAX-1) begin
                rx_len <= rx_len + 8'd1;
              end
          end
          if (frame_end) begin
            st <= S_PARSE;
          end
        end

        // =========================
        S_PARSE: begin
          if (!rx_in_ascii) begin
            // --- RTU: [addr][func][...][CRCL][CRCH]
            if (rx_len < 4) begin
              st <= S_IDLE;
            end else begin
              dev_addr <= rx_buf[0];
              func     <= rx_buf[1];
              // validate CRC over 0..rx_len-3
              if (crc16_sw(rx_len-2) != {rx_buf[rx_len-1], rx_buf[rx_len-2]}) begin
                stat_crc_err <= 1'b1;
                st <= S_IDLE;
              end else begin
                // parse common fields if present
                addr <= {rx_buf[2], rx_buf[3]};
                if (func==8'h01 || func==8'h02 || func==8'h03 || func==8'h04) begin
                  qty <= {rx_buf[4], rx_buf[5]};
                end else if (func==8'h05 || func==8'h06) begin
                  val <= {rx_buf[4], rx_buf[5]};
                end else if (func==8'h0F || func==8'h10) begin
                  qty        <= {rx_buf[4], rx_buf[5]};
                  byte_count <= rx_buf[6];
                end
                st <= S_EXEC;
              end
            end
          end else begin
            // --- ASCII: ":" ... LRC "\r\n" -> decode hex pairs into bytes for bin[]
            blen      = 0;
            ascii_err = 1'b0;

            // Expect rx_buf[0] = ':' ; data from [1 .. rx_len-3], LRC at last byte before CRLF
            for (i=1; i<BUF_MAX-3; i=i+2) begin
                if ((i+1) < ({24'd0, rx_len} - 32'd2)) begin
                nb0 = hex2nib(rx_buf[i[7:0]]);
                nb1 = hex2nib(rx_buf[(i+1) & (BUF_MAX-1)]);
                if (nb0==8'hFF || nb1==8'hFF) begin
                  ascii_err = 1'b1;
                end else begin
                  b = {nb0[3:0], nb1[3:0]};
                  bin[blen[7:0]] = b;
                  blen = blen + 1;
                end
              end
            end

            if (ascii_err || blen < 3) begin
              stat_lrc_err <= ascii_err;
              st <= S_IDLE;
            end else begin
              // copy into rx_buf (binary)
              for (k=0; k<BUF_MAX; k=k+1) begin
                if (k < blen) rx_buf[k[7:0]] = bin[k[7:0]];
              end
              // Use non-blocking assignment for sequential consistency
              rx_len <= blen[7:0];

              // verify LRC: sum of bytes including address..data..LRC == 0
              sum = 8'h00;
              for (k=0; k<BUF_MAX; k=k+1) begin
                if (k < blen) sum = sum + rx_buf[k[7:0]];
              end

              if (sum != 8'h00) begin
                stat_lrc_err <= 1'b1;
                st <= S_IDLE;
              end else begin
                dev_addr <= rx_buf[0];
                func     <= rx_buf[1];
                addr     <= {rx_buf[2], rx_buf[3]};
                if (func==8'h01 || func==8'h02 || func==8'h03 || func==8'h04) begin
                  qty <= {rx_buf[4], rx_buf[5]};
                end else if (func==8'h05 || func==8'h06) begin
                  val <= {rx_buf[4], rx_buf[5]};
                end else if (func==8'h0F || func==8'h10) begin
                  qty        <= {rx_buf[4], rx_buf[5]};
                  byte_count <= rx_buf[6];
                end
                st <= S_EXEC;
              end
            end
          end
        end

        // =========================
        S_EXEC: begin
          // Slave: execute request and build response.
          // Master: treat inbound as response; update images.
          if (!cfg_mode_master) begin
            // ---------- SLAVE ----------
              if (func==8'h01 || func==8'h02) begin
                // read coils/discretes
                  bytes  = ({16'd0, qty} + 32'd7) >> 3;
                tx_wr   = 0;

                // resp: addr func bytecount data... crc
                tx_buf[tx_wr[7:0]] = dev_addr; tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = func;     tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = bytes[7:0]; tx_wr = tx_wr + 1;

                for (i=0; i<BUF_MAX; i=i+1) begin
                  if (i < bytes) begin
                    b = 8'h00;
                    for (bitn=0; bitn<8; bitn=bitn+1) begin
                      if ((i*8+bitn) < qty) begin
                        shamt = {16'd0, addr} + i*8 + bitn;
                        if (func==8'h01) b[bitn] = (((do_wdata >> shamt) & 32'h1) != 32'h0);
                        else             b[bitn] = (((di_status >> shamt) & 32'h1) != 32'h0);
                      end
                    end
                    tx_buf[tx_wr[7:0]] = b; tx_wr = tx_wr + 1;
                  end
                end

                // CRC
                c = 16'hFFFF;
                for (j=0; j<BUF_MAX; j=j+1) begin
                  if (j < tx_wr) begin
                    // Extend byte to 16 bits for XOR to avoid width mismatch
                    x = c ^ {8'h00, tx_buf[j]};
                    for (k=0; k<8; k=k+1) begin
                      if (x[0]) x = (x>>1) ^ 16'hA001; else x = (x>>1);
                    end
                    c = x;
                  end
                end
                tx_buf[tx_wr[7:0]] = c[7:0];  tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = c[15:8]; tx_wr = tx_wr + 1;

                tx_len <= tx_wr[8:0];
                st <= S_SEND; tx_idx <= 9'd0;

              end else if (func==8'h03 || func==8'h04) begin
                // read holding/input regs
                tx_wr   = 0;
                tx_buf[tx_wr[7:0]] = dev_addr; tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = func;     tx_wr = tx_wr + 1;

                  // Shift and mask to 8 bits
                  tmp8 = qty[7:0] << 1;
                tx_buf[tx_wr[7:0]] = tmp8; tx_wr = tx_wr + 1;

                for (i=0; i<REG_WORDS; i=i+1) begin
                  if (i < qty) begin
                    ofs = {16'd0, addr} + i - {16'd0, cfg_map_base_qw_iw};
                    if (func==8'h03) w = reg_holding[ofs[5:0]];
                    else             w = reg_input  [ofs[5:0]];
                    tx_buf[tx_wr[7:0]] = w[15:8]; tx_wr = tx_wr + 1;
                    tx_buf[tx_wr[7:0]] = w[7:0];  tx_wr = tx_wr + 1;
                  end
                end

                // CRC
                c = 16'hFFFF;
                for (j=0; j<BUF_MAX; j=j+1) begin
                  if (j < tx_wr) begin
                    // Extend byte to 16 bits for XOR to avoid width mismatch
                    x = c ^ {8'h00, tx_buf[j]};
                    for (k=0; k<8; k=k+1) x = x[0] ? ((x>>1)^16'hA001) : (x>>1);
                    c = x;
                  end
                end
                tx_buf[tx_wr[7:0]] = c[7:0];  tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = c[15:8]; tx_wr = tx_wr + 1;

                tx_len <= tx_wr[8:0];
                st <= S_SEND; tx_idx <= 9'd0;

              end else if (func==8'h05) begin
                // write single coil: echo request
                do_wmask <= (32'h1 << addr[4:0]);
                do_wdata <= (val==16'hFF00) ? (32'h1 << addr[4:0]) : 32'h0;
                do_we    <= 1'b1;

                tx_wr   = 0;
                for (j=0; j<6; j=j+1) begin
                  tx_buf[tx_wr[7:0]] = rx_buf[j]; tx_wr = tx_wr + 1;
                end

                c = 16'hFFFF;
                for (j=0; j<BUF_MAX; j=j+1) begin
                  if (j < tx_wr) begin
                    x = c ^ {8'h00, tx_buf[j]};
                    for (k=0; k<8; k=k+1) x = x[0] ? ((x>>1)^16'hA001) : (x>>1);
                    c = x;
                  end
                end
                tx_buf[tx_wr[7:0]] = c[7:0];  tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = c[15:8]; tx_wr = tx_wr + 1;

                tx_len <= tx_wr[8:0];
                st <= S_SEND; tx_idx <= 9'd0;

              end else if (func==8'h06) begin
                // write single holding reg
                ofs = {16'd0, addr} - {16'd0, cfg_map_base_qw_iw};
                reg_holding[ofs[5:0]] <= val;

                tx_wr   = 0;
                for (j=0; j<6; j=j+1) begin
                  tx_buf[tx_wr[7:0]] = rx_buf[j]; tx_wr = tx_wr + 1;
                end

                c = 16'hFFFF;
                for (j=0; j<BUF_MAX; j=j+1) begin
                  if (j < tx_wr) begin
                    x = c ^ {8'h00, tx_buf[j]};
                    for (k=0; k<8; k=k+1) x = x[0] ? ((x>>1)^16'hA001) : (x>>1);
                    c = x;
                  end
                end
                tx_buf[tx_wr[7:0]] = c[7:0];  tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = c[15:8]; tx_wr = tx_wr + 1;

                tx_len <= tx_wr[8:0];
                st <= S_SEND; tx_idx <= 9'd0;

              end else if (func==8'h0F) begin
                // write multiple coils: unpack data starting rx_buf[7..7+byte_count-1]
                new_do = 32'h0;
                msk    = 32'h0;
                  for (i=0; i<32; i=i+1) begin
                    if (i < byte_count) begin
                      for (bitn=0; bitn<8; bitn=bitn+1) begin
                        if ((i*8+bitn) < qty) begin
                          shamt = {16'd0, addr} + i*8 + bitn;
                          msk[shamt[4:0]]    = 1'b1;
                          new_do[shamt[4:0]] = rx_buf[7+i][bitn];
                        end
                      end
                    end
                  end
                do_wmask <= msk;
                do_wdata <= new_do;
                do_we    <= 1'b1;

                // Response: echo addr func start qty
                tx_wr   = 0;
                tx_buf[tx_wr[7:0]] = dev_addr; tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = func;     tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = rx_buf[2]; tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = rx_buf[3]; tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = rx_buf[4]; tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = rx_buf[5]; tx_wr = tx_wr + 1;

                c = 16'hFFFF;
                for (j=0; j<BUF_MAX; j=j+1) begin
                  if (j < tx_wr) begin
                    x = c ^ {8'h00, tx_buf[j]};
                    for (k=0; k<8; k=k+1) x = x[0] ? ((x>>1)^16'hA001) : (x>>1);
                    c = x;
                  end
                end
                tx_buf[tx_wr[7:0]] = c[7:0];  tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = c[15:8]; tx_wr = tx_wr + 1;

                tx_len <= tx_wr[8:0];
                st <= S_SEND; tx_idx <= 9'd0;

              end else if (func==8'h10) begin
                // write multiple holding regs: unpack words from rx_buf[7..]
                for (i=0; i<REG_WORDS; i=i+1) begin
                  if (i < qty) begin
                      w = {rx_buf[7+i*2], rx_buf[8+i*2]};
                      ofs = {16'd0, addr} + i - {16'd0, cfg_map_base_qw_iw};
                      reg_holding[ofs[5:0]] <= w;
                  end
                end

                // Response: echo addr func start qty
                tx_wr   = 0;
                tx_buf[tx_wr[7:0]] = dev_addr; tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = func;     tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = rx_buf[2]; tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = rx_buf[3]; tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = rx_buf[4]; tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = rx_buf[5]; tx_wr = tx_wr + 1;

                c = 16'hFFFF;
                for (j=0; j<BUF_MAX; j=j+1) begin
                  if (j < tx_wr) begin
                    x = c ^ {8'h00, tx_buf[j]};
                    for (k=0; k<8; k=k+1) x = x[0] ? ((x>>1)^16'hA001) : (x>>1);
                    c = x;
                  end
                end
                tx_buf[tx_wr[7:0]] = c[7:0];  tx_wr = tx_wr + 1;
                tx_buf[tx_wr[7:0]] = c[15:8]; tx_wr = tx_wr + 1;

                tx_len <= tx_wr[8:0];
                st <= S_SEND; tx_idx <= 9'd0;

            end else begin
              st <= S_IDLE;
            end
          end else begin
            // ---------- MASTER (handle responses) ----------
            if (func==8'h01 || func==8'h02) begin
              bc  = rx_buf[2];
              ofs = {16'd0, scan_rbase};
              st <= S_IDLE;
            end

              else if (func==8'h03 || func==8'h04) begin
                  base = {16'd0, scan_rbase};
                for (i=0; i<REG_WORDS; i=i+1) begin
                  if (i < qty) begin
                    w = {rx_buf[3+i*2], rx_buf[4+i*2]};
                    reg_input[base + i] <= w;
                  end
                end
                st <= S_IDLE;

            end else begin
              st <= S_IDLE;
            end
          end
        end

        // =========================
          S_SEND: begin
            if (tx_b_rdy && (tx_idx < tx_len)) begin
              tx_b   <= tx_buf[tx_idx[7:0]];
              tx_b_v <= 1'b1;
              tx_idx <= tx_idx + 9'd1;
            end
            if (tx_idx == tx_len) begin
              st <= S_IDLE;
            end
          end

        default: begin
          st <= S_IDLE;
        end
      endcase
    end
  end
endmodule
