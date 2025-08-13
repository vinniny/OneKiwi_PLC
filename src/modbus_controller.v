// modbus_controller.v (Verilog-2001 compatible)
// - Master/Slave handling
// - Parses RTU/ASCII frames (ASCII bytes passed through from bridge; hex decode inside)
// - Executes FC 01/02/03/04/05/06/0F/10

module modbus_controller #(
  parameter REG_WORDS = 64,
  parameter SCAN_MAX  = 16
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
  reg [7:0] rx_buf [0:255];
  reg [7:0] rx_len;
  reg       rx_in_ascii;

  // ===== TX buffer (per-frame) =====
  reg [7:0] tx_buf [0:255];
  reg [7:0] tx_len;
  reg [7:0] tx_idx;

  // ===== Shared temporaries =====
  integer i,j,k,bitn,ofs,base;
  integer bytes, blen;
  reg [7:0] b, nb0, nb1, bc, sum;
  reg [7:0] bin [0:255];          // temp for ASCII hex -> binary
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
    input integer n;
    integer i0,j0; reg [15:0] c0; reg [7:0] d0;
    begin
      c0        = 16'hFFFF;
      crc16_sw  = 16'h0000;
      for (i0=0;i0<n;i0=i0+1) begin
        d0 = rx_buf[i0];
        for (j0=0;j0<8;j0=j0+1) begin
          if (((c0^d0) & 16'h0001)!=16'h0000) c0=(c0>>1)^16'hA001; else c0=(c0>>1);
          d0 = d0 >> 1;
        end
      end
      crc16_sw = c0;
    end
  endfunction

  function [7:0] hex2nib; input [7:0] c_in;
    begin
      if (c_in>="0" && c_in<="9") hex2nib=c_in-"0";
      else if (c_in>="A" && c_in<="F") hex2nib=c_in-"A"+8'd10;
      else if (c_in>="a" && c_in<="f") hex2nib=c_in-"a"+8'd10;
      else hex2nib=8'hFF;
    end
  endfunction

  // ===== Main sequential block =====
  always @(posedge clk) begin
    if (rst) begin
      st <= S_IDLE;
      rx_len <= 8'd0;
      tx_idx <= 8'd0;
      tx_len <= 8'd0;
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
            if (rx_len != 8'hFF) begin
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
            for (i=1; (i+1) < (rx_len-2); i=i+2) begin
              nb0 = hex2nib(rx_buf[i]);
              nb1 = hex2nib(rx_buf[i+1]);
              if (nb0==8'hFF || nb1==8'hFF) begin
                ascii_err = 1'b1;
              end else begin
                b = {nb0, nb1};
                bin[blen] = b;
                blen = blen + 1;
              end
            end

            if (ascii_err || blen < 3) begin
              stat_lrc_err <= ascii_err;
              st <= S_IDLE;
            end else begin
              // copy into rx_buf (binary)
              for (k=0; k<blen; k=k+1) begin
                rx_buf[k] = bin[k];
              end
              rx_len = blen;

              // verify LRC: sum of bytes including address..data..LRC == 0
              sum = 8'h00;
              for (k=0; k<blen; k=k+1) begin
                sum = sum + rx_buf[k];
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
              bytes  = (qty + 16'd7) >> 3;
              tx_len = 8'd0;

              // resp: addr func bytecount data... crc
              tx_buf[tx_len] = dev_addr; tx_len = tx_len + 1;
              tx_buf[tx_len] = func;     tx_len = tx_len + 1;
              tx_buf[tx_len] = bytes[7:0]; tx_len = tx_len + 1;

              for (i=0; i<bytes; i=i+1) begin
                b = 8'h00;
                for (bitn=0; bitn<8; bitn=bitn+1) begin
                  if ((i*8+bitn) < qty) begin
                    if (func==8'h01) b[bitn] = ((do_wdata >> (addr + i*8 + bitn)) & 1'b1);
                    else             b[bitn] = ((di_status >> (addr + i*8 + bitn)) & 1'b1);
                  end
                end
                tx_buf[tx_len] = b; tx_len = tx_len + 1;
              end

              // CRC
              c = 16'hFFFF;
              for (j=0; j<tx_len; j=j+1) begin
                x = c ^ tx_buf[j];
                for (k=0; k<8; k=k+1) begin
                  if (x[0]) x = (x>>1) ^ 16'hA001; else x = (x>>1);
                end
                c = x;
              end
              tx_buf[tx_len] = c[7:0];  tx_len = tx_len + 1;
              tx_buf[tx_len] = c[15:8]; tx_len = tx_len + 1;

              st <= S_SEND; tx_idx <= 8'd0;

            end else if (func==8'h03 || func==8'h04) begin
              // read holding/input regs
              tx_len = 8'd0;
              tx_buf[tx_len] = dev_addr; tx_len = tx_len + 1;
              tx_buf[tx_len] = func;     tx_len = tx_len + 1;

              tmp8 = (qty << 1);                 // avoid (qty<<1)[7:0]
              tx_buf[tx_len] = tmp8; tx_len = tx_len + 1;

              for (i=0; i<qty; i=i+1) begin
                if (func==8'h03) w = reg_holding[(addr + i) - cfg_map_base_qw_iw];
                else             w = reg_input  [(addr + i) - cfg_map_base_qw_iw];
                tx_buf[tx_len] = w[15:8]; tx_len = tx_len + 1;
                tx_buf[tx_len] = w[7:0];  tx_len = tx_len + 1;
              end

              // CRC
              c = 16'hFFFF;
              for (j=0; j<tx_len; j=j+1) begin
                x = c ^ tx_buf[j];
                for (k=0; k<8; k=k+1) x = x[0] ? ((x>>1)^16'hA001) : (x>>1);
                c = x;
              end
              tx_buf[tx_len] = c[7:0];  tx_len = tx_len + 1;
              tx_buf[tx_len] = c[15:8]; tx_len = tx_len + 1;

              st <= S_SEND; tx_idx <= 8'd0;

            end else if (func==8'h05) begin
              // write single coil: echo request
              do_wmask <= (32'h1 << addr[4:0]);
              do_wdata <= (val==16'hFF00) ? (32'h1 << addr[4:0]) : 32'h0;
              do_we    <= 1'b1;

              tx_len = 8'd0;
              for (j=0; j<6; j=j+1) begin
                tx_buf[tx_len] = rx_buf[j]; tx_len = tx_len + 1;
              end

              c = 16'hFFFF;
              for (j=0; j<tx_len; j=j+1) begin
                x = c ^ tx_buf[j];
                for (k=0; k<8; k=k+1) x = x[0] ? ((x>>1)^16'hA001) : (x>>1);
                c = x;
              end
              tx_buf[tx_len] = c[7:0];  tx_len = tx_len + 1;
              tx_buf[tx_len] = c[15:8]; tx_len = tx_len + 1;

              st <= S_SEND; tx_idx <= 8'd0;

            end else if (func==8'h06) begin
              // write single holding reg
              reg_holding[addr - cfg_map_base_qw_iw] <= val;

              tx_len = 8'd0;
              for (j=0; j<6; j=j+1) begin
                tx_buf[tx_len] = rx_buf[j]; tx_len = tx_len + 1;
              end

              c = 16'hFFFF;
              for (j=0; j<tx_len; j=j+1) begin
                x = c ^ tx_buf[j];
                for (k=0; k<8; k=k+1) x = x[0] ? ((x>>1)^16'hA001) : (x>>1);
                c = x;
              end
              tx_buf[tx_len] = c[7:0];  tx_len = tx_len + 1;
              tx_buf[tx_len] = c[15:8]; tx_len = tx_len + 1;

              st <= S_SEND; tx_idx <= 8'd0;

            end else if (func==8'h0F) begin
              // write multiple coils: unpack data starting rx_buf[7..7+byte_count-1]
              new_do = 32'h0;
              msk    = 32'h0;
              for (i=0; i<byte_count; i=i+1) begin
                for (bitn=0; bitn<8; bitn=bitn+1) begin
                  if ((i*8+bitn) < qty) begin
                    msk[addr + i*8 + bitn]    = 1'b1;
                    new_do[addr + i*8 + bitn] = rx_buf[7+i][bitn];
                  end
                end
              end
              do_wmask <= msk;
              do_wdata <= new_do;
              do_we    <= 1'b1;

              // Response: echo addr func start qty
              tx_len = 8'd0;
              tx_buf[tx_len] = dev_addr; tx_len = tx_len + 1;
              tx_buf[tx_len] = func;     tx_len = tx_len + 1;
              tx_buf[tx_len] = rx_buf[2]; tx_len = tx_len + 1;
              tx_buf[tx_len] = rx_buf[3]; tx_len = tx_len + 1;
              tx_buf[tx_len] = rx_buf[4]; tx_len = tx_len + 1;
              tx_buf[tx_len] = rx_buf[5]; tx_len = tx_len + 1;

              c = 16'hFFFF;
              for (j=0; j<tx_len; j=j+1) begin
                x = c ^ tx_buf[j];
                for (k=0; k<8; k=k+1) x = x[0] ? ((x>>1)^16'hA001) : (x>>1);
                c = x;
              end
              tx_buf[tx_len] = c[7:0];  tx_len = tx_len + 1;
              tx_buf[tx_len] = c[15:8]; tx_len = tx_len + 1;

              st <= S_SEND; tx_idx <= 8'd0;

            end else if (func==8'h10) begin
              // write multiple holding regs: unpack words from rx_buf[7..]
              for (i=0; i<qty; i=i+1) begin
                w = {rx_buf[7+i*2], rx_buf[8+i*2]};
                reg_holding[(addr + i) - cfg_map_base_qw_iw] <= w;
              end

              // Response: echo addr func start qty
              tx_len = 8'd0;
              tx_buf[tx_len] = dev_addr; tx_len = tx_len + 1;
              tx_buf[tx_len] = func;     tx_len = tx_len + 1;
              tx_buf[tx_len] = rx_buf[2]; tx_len = tx_len + 1;
              tx_buf[tx_len] = rx_buf[3]; tx_len = tx_len + 1;
              tx_buf[tx_len] = rx_buf[4]; tx_len = tx_len + 1;
              tx_buf[tx_len] = rx_buf[5]; tx_len = tx_len + 1;

              c = 16'hFFFF;
              for (j=0; j<tx_len; j=j+1) begin
                x = c ^ tx_buf[j];
                for (k=0; k<8; k=k+1) x = x[0] ? ((x>>1)^16'hA001) : (x>>1);
                c = x;
              end
              tx_buf[tx_len] = c[7:0];  tx_len = tx_len + 1;
              tx_buf[tx_len] = c[15:8]; tx_len = tx_len + 1;

              st <= S_SEND; tx_idx <= 8'd0;

            end else begin
              st <= S_IDLE;
            end
          end else begin
            // ---------- MASTER (handle responses) ----------
            if (func==8'h01 || func==8'h02) begin
              bc  = rx_buf[2];
              ofs = scan_rbase;
              st <= S_IDLE;
            end

            else if (func==8'h03 || func==8'h04) begin
              base = scan_rbase;
              for (i=0; i<qty; i=i+1) begin
                w = {rx_buf[3+i*2], rx_buf[4+i*2]};
                reg_input[base + i] <= w;
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
            tx_b   <= tx_buf[tx_idx];
            tx_b_v <= 1'b1;
            tx_idx <= tx_idx + 8'd1;
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
