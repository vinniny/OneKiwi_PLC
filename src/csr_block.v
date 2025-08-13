// csr_block.v - APB3 CSR block for Modbus Converter IP
// Sync reset design. Zero-wait PREADY. W1C for status.
// Adds scan-table CSRs for master-mode autonomous polling.
//
// Map (word offsets):
// 0x00 REG00 DO (%QX)                RW
// 0x04 REG01 DI (%IX)                RO
// 0x08 REG02 TIMER                   RW (free-running)
// 0x0C REG03 MSG (TX push / RX pop)  WO/RO  [7:0] data
// 0x10 REG04 CFG0                    RW  [0]=mode(1=Master,0=Slave)
//                                      [2:1]=parity(0=None,1=Even,2=Odd)
//                                      [4]=stop(0=1stop,1=2stop)
//                                      [8]=ASCII enable
//                                      [15:9] res
//                                      [31:16]=rtu_sil_q88
// 0x14 REG05 CFG1                    RW  [15:0]=baud_div(×16)
//                                      [31:16]=msg_wm
// 0x18 REG06 MAP_BASE_QW_IW          RW  [15:0]=base index for %QW/%IW
// 0x1C REG07 IRQ/STATUS              RW1C [0]=rx_avail [1]=tx_empty [2]=crc_err
//                                          [3]=lrc_err [4]=frame_to
// Scan Engine (optional, master mode):
// 0x20 REG08_SCAN_CTRL               RW  [0]=scan_en [7:4]=retry_max
//                                      [23:8]=scan_period_ms
// 0x24 REG09_SCAN_CNT                RO  [15:0]=cycles_done [31:16]=err_count
// 0x28 REG0A_SCAN_IDX                RW  index 0..(SCAN_MAX-1) for entry access
// 0x2C REG0B_SCAN_ENTRY              RW  [7:0]=slave [15:8]=func
//                                      [31:16]=start_addr
// 0x30 REG0C_SCAN_QTY                RW  [15:0]=qty [31:16]=byte_count (for 0F/10)
// 0x34 REG0D_SCAN_WBASE              RW  [15:0]=%Q base index for writes
// 0x38 REG0E_SCAN_RBASE              RW  [15:0]=%I base index for reads
//
// NOTE: The scan-table shadow regs here feed 'scan_engine' inside controller/top.

module csr_block #(
  parameter ADDR_WIDTH = 12,
  parameter SCAN_MAX   = 16
)(
  input  wire        PCLK,
  input  wire        PRESETn,     // external async; sync internally

  input  wire [ADDR_WIDTH-1:0] PADDR,
  input  wire        PSEL,
  input  wire        PENABLE,
  input  wire        PWRITE,
  input  wire [31:0] PWDATA,
  input  wire [3:0]  PSTRB,
  output reg  [31:0] PRDATA,
  output reg         PREADY,
  output wire        PSLVERR,

  // DO/DI
  input  wire [31:0] di_status_in,
  output reg  [31:0] do_control_out,

  // Timer
  output      [31:0] timer_cnt,

  // Host message buffer (byte streaming interface)
  input  wire [7:0]  rx_data,
  input  wire        rx_valid,
  output reg         rx_pop,
  output reg  [7:0]  tx_data,
  output reg         tx_push,
  input  wire        tx_ready,

  // Config outs
  output reg         cfg_mode_master,
  output reg  [1:0]  cfg_parity,
  output reg         cfg_stop2,
  output reg         cfg_ascii_en,
  output reg [15:0]  cfg_rtu_sil_q88,
  output reg [15:0]  cfg_baud_div,
  output reg [15:0]  cfg_msg_wm,
  output reg [15:0]  cfg_map_base_qw_iw,

  // IRQ/status in
  input  wire        stat_crc_err,
  input  wire        stat_lrc_err,
  input  wire        stat_frame_to,
  input  wire        stat_tx_empty,
  input  wire        stat_rx_avail,

  // W1C echo
  output reg         irq_w1c,

  // Scan table reflection
  output reg         scan_en,
  output reg  [3:0]  scan_retry_max,
  output reg [15:0]  scan_period_ms,
  input  wire [15:0] scan_cycles_done,
  input  wire [15:0] scan_err_count,

  output reg  [7:0]  scan_idx,
  output reg  [7:0]  scan_slave,
  output reg  [7:0]  scan_func,
  output reg [15:0]  scan_start_addr,
  output reg [15:0]  scan_qty,
  output reg [15:0]  scan_byte_count,
  output reg [15:0]  scan_wbase,
  output reg [15:0]  scan_rbase
);

  // sync reset
  reg [1:0] rff;
  wire rst;
  always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) rff <= 2'b00;
    else          rff <= {rff[0],1'b1};
  end
  assign rst = ~rff[1];

  assign PSLVERR = 1'b0;

  // internal regs
  reg [31:0] reg00_do, reg01_di, reg02_timer, reg04_cfg0, reg05_cfg1, reg06_map, reg07_irq;
  reg [31:0] reg08_scan_ctrl, reg09_scan_cnt, reg0a_scan_idx, reg0b_scan_entry, reg0c_scan_qty, reg0d_scan_wbase, reg0e_scan_rbase;

  // PREADY (zero-wait)
  always @(posedge PCLK) begin
    if (rst) PREADY <= 1'b0;
    else     PREADY <= (PSEL & ~PREADY);
  end

  wire wr = PSEL & PENABLE & PWRITE & PREADY;
  wire rd = PSEL & PENABLE & ~PWRITE & PREADY;

// Timer + DI mirror  (single writer for reg02_timer)
always @(posedge PCLK) begin
  if (rst) begin
    reg01_di    <= 32'h0;
    reg02_timer <= 32'h0;
  end else begin
    reg01_di    <= di_status_in;

    // APB write to REG02 takes priority over increment this cycle
    if (wr && (PADDR[5:2]==4'h2)) begin
      // byte-strobe write (avoid wb() to keep this block self-contained)
      if (PSTRB[0]) reg02_timer[ 7:0]  <= PWDATA[ 7:0];
      if (PSTRB[1]) reg02_timer[15:8]  <= PWDATA[15:8];
      if (PSTRB[2]) reg02_timer[23:16] <= PWDATA[23:16];
      if (PSTRB[3]) reg02_timer[31:24] <= PWDATA[31:24];
    end else begin
      reg02_timer <= reg02_timer + 32'd1;
    end
  end
end
assign timer_cnt = reg02_timer;


  // IRQ status (W1C)
  always @(posedge PCLK) begin
    if (rst) begin
      reg07_irq <= 32'h0;
      irq_w1c   <= 1'b0;
    end else begin
      reg07_irq[0] <= stat_rx_avail;
      reg07_irq[1] <= stat_tx_empty;
      reg07_irq[2] <= stat_crc_err;
      reg07_irq[3] <= stat_lrc_err;
      reg07_irq[4] <= stat_frame_to;
      if (wr && (PADDR[5:2]==4'h7)) begin
        reg07_irq <= reg07_irq & ~PWDATA; // W1C
        irq_w1c   <= 1'b1;
      end else irq_w1c <= 1'b0;
    end
  end

  // Writes
  function [31:0] wb;
    input [31:0] oldv, newv; input [3:0] st;
    begin
      wb = oldv;
      if (st[0]) wb[ 7:0 ] = newv[ 7:0 ];
      if (st[1]) wb[15:8 ] = newv[15:8 ];
      if (st[2]) wb[23:16] = newv[23:16];
      if (st[3]) wb[31:24] = newv[31:24];
    end
  endfunction

  always @(posedge PCLK) begin
    if (rst) begin
      reg00_do <= 32'h0;
      reg04_cfg0 <= 32'h0001_0000; // Master=1, parity=None, stop=1, ASCII=0, sil=1
      reg05_cfg1 <= 32'h0080_0036; // wm=0x80, baud_div=54 (~115200 @ 100MHz/16)
      reg06_map  <= 32'h0;
      tx_push    <= 1'b0; rx_pop <= 1'b0; tx_data<=8'h00;

      // scan defaults
      reg08_scan_ctrl <= 32'h0001_0014; // en=1, retry_max=1, period=20ms
      reg09_scan_cnt  <= 32'h0;
      reg0a_scan_idx  <= 32'h0;
      reg0b_scan_entry<= 32'h0001_0400; // slave=1, func=4, start=0
      reg0c_scan_qty  <= 32'h0010_0010; // qty=16, byte_count=16 (for regs)
      reg0d_scan_wbase<= 32'h0;
      reg0e_scan_rbase<= 32'h0;
    end else begin
      tx_push <= 1'b0; rx_pop <= 1'b0;

      if (wr) begin
        case (PADDR[5:2])
          4'h0: reg00_do <= wb(reg00_do, PWDATA, PSTRB);
         
          4'h3: if (tx_ready) begin tx_data <= PWDATA[7:0]; tx_push<=1'b1; end
          4'h4: reg04_cfg0 <= wb(reg04_cfg0, PWDATA, PSTRB);
          4'h5: reg05_cfg1 <= wb(reg05_cfg1, PWDATA, PSTRB);
          4'h6: reg06_map  <= wb(reg06_map , PWDATA, PSTRB);
          4'h7: ; // W1C handled
          4'h8: reg08_scan_ctrl <= wb(reg08_scan_ctrl, PWDATA, PSTRB);
          4'hA: reg0a_scan_idx  <= wb(reg0a_scan_idx , PWDATA, PSTRB);
          4'hB: reg0b_scan_entry<= wb(reg0b_scan_entry, PWDATA, PSTRB);
          4'hC: reg0c_scan_qty  <= wb(reg0c_scan_qty , PWDATA, PSTRB);
          4'hD: reg0d_scan_wbase<= wb(reg0d_scan_wbase, PWDATA, PSTRB);
          4'hE: reg0e_scan_rbase<= wb(reg0e_scan_rbase, PWDATA, PSTRB);
          default: ;
        endcase
      end

      if (rd && (PADDR[5:2]==4'h3)) begin
        if (rx_valid) rx_pop <= 1'b1;
      end

      // read-only counters mirror
      reg09_scan_cnt[15:0]  <= scan_cycles_done;
      reg09_scan_cnt[31:16] <= scan_err_count;
    end
  end

  // Read MUX
  always @(*) begin
    case (PADDR[5:2])
      4'h0: PRDATA = reg00_do;
      4'h1: PRDATA = reg01_di;
      4'h2: PRDATA = reg02_timer;
      4'h3: PRDATA = {24'h0, rx_data};
      4'h4: PRDATA = reg04_cfg0;
      4'h5: PRDATA = reg05_cfg1;
      4'h6: PRDATA = reg06_map;
      4'h7: PRDATA = reg07_irq;
      4'h8: PRDATA = reg08_scan_ctrl;
      4'h9: PRDATA = reg09_scan_cnt;
      4'hA: PRDATA = reg0a_scan_idx;
      4'hB: PRDATA = reg0b_scan_entry;
      4'hC: PRDATA = reg0c_scan_qty;
      4'hD: PRDATA = reg0d_scan_wbase;
      4'hE: PRDATA = reg0e_scan_rbase;
      default: PRDATA = 32'h0;
    endcase
  end

  // Export configs
  always @(*) begin
    cfg_mode_master   = reg04_cfg0[0];
    cfg_parity        = reg04_cfg0[2:1];
    cfg_stop2         = reg04_cfg0[4];
    cfg_ascii_en      = reg04_cfg0[8];
    cfg_rtu_sil_q88   = reg04_cfg0[31:16];
    cfg_baud_div      = reg05_cfg1[15:0];
    cfg_msg_wm        = reg05_cfg1[31:16];
    cfg_map_base_qw_iw= reg06_map[15:0];
    do_control_out    = reg00_do;

    // scan
    scan_en        = reg08_scan_ctrl[0];
    scan_retry_max = reg08_scan_ctrl[7:4];
    scan_period_ms = reg08_scan_ctrl[23:8];

    scan_idx       = reg0a_scan_idx[7:0];
    scan_slave     = reg0b_scan_entry[7:0];
    scan_func      = reg0b_scan_entry[15:8];
    scan_start_addr= reg0b_scan_entry[31:16];
    scan_qty       = reg0c_scan_qty[15:0];
    scan_byte_count= reg0c_scan_qty[31:16];
    scan_wbase     = reg0d_scan_wbase[15:0];
    scan_rbase     = reg0e_scan_rbase[15:0];
  end

endmodule
