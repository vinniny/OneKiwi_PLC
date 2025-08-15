// top_modbus_converter.v - Integrates CSR, UART bridge, GPIO, and controller
module top_modbus_converter #(
  parameter REG_WORDS = 64,
  parameter SCAN_MAX  = 16
)(
  input  wire        PCLK,
  input  wire        PRESETn,

  // APB3
  input  wire [11:0] PADDR,
  input  wire        PSEL,
  input  wire        PENABLE,
  input  wire        PWRITE,
  input  wire [31:0] PWDATA,
  input  wire [3:0]  PSTRB,
  output wire [31:0] PRDATA,
  output wire        PREADY,
  output wire        PSLVERR,

  // UART pins
  input  wire        UART_RX,
  output wire        UART_TX,

  // GPIOs
  input  wire [31:0] GPIO_DI,
  output wire [31:0] GPIO_DO
);
  // sync reset
  reg [1:0] rff;
  always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn) rff<=2'b00; else rff<={rff[0],1'b1};
  end
  wire rst = ~rff[1];

  // Wires between blocks
  wire [31:0] di_status;
  /* verilator lint_off UNUSED */
  wire [31:0] do_control;
  wire [31:0] timer_cnt;
  wire        csr_rx_pop;
  wire [7:0]  csr_tx_data;
  wire        csr_tx_push;
  wire [15:0] cfg_msg_wm;
  wire        irq_w1c;
  /* verilator lint_on UNUSED */

  wire [7:0]  csr_rx_data;
  wire        csr_rx_valid;
  wire        csr_tx_ready;

  wire        cfg_mode_master;
  wire [1:0]  cfg_parity;
  wire        cfg_stop2;
  wire        cfg_ascii_en;
  wire [15:0] cfg_rtu_sil_q88;
  wire [15:0] cfg_baud_div;
  wire [15:0] cfg_map_base_qw_iw;

  wire        stat_crc_err, stat_lrc_err, stat_frame_to;
  reg         stat_tx_empty;

  always @(posedge PCLK) begin
    if (rst)
      stat_tx_empty <= 1'b1;
    else
      stat_tx_empty <= 1'b1;
  end

  // Scan CSRs
  wire        scan_en;
  wire [3:0]  scan_retry_max;
  wire [15:0] scan_period_ms;
  wire [7:0]  scan_idx;
  wire [7:0]  scan_slave;
  wire [7:0]  scan_func;
  wire [15:0] scan_start_addr, scan_qty, scan_byte_count, scan_wbase, scan_rbase;
  wire [15:0] scan_cycles_done, scan_err_count;

  csr_block #(.SCAN_MAX(SCAN_MAX)) u_csr(
    .PCLK(PCLK), .PRESETn(PRESETn),
    .PADDR(PADDR), .PSEL(PSEL), .PENABLE(PENABLE), .PWRITE(PWRITE),
    .PWDATA(PWDATA), .PSTRB(PSTRB), .PRDATA(PRDATA), .PREADY(PREADY), .PSLVERR(PSLVERR),
    .di_status_in(di_status), .do_control_out(do_control),
    .timer_cnt(timer_cnt),
    .rx_data(csr_rx_data), .rx_valid(csr_rx_valid), .rx_pop(csr_rx_pop),
    .tx_data(csr_tx_data), .tx_push(csr_tx_push), .tx_ready(csr_tx_ready),
    .cfg_mode_master(cfg_mode_master), .cfg_parity(cfg_parity), .cfg_stop2(cfg_stop2),
    .cfg_ascii_en(cfg_ascii_en), .cfg_rtu_sil_q88(cfg_rtu_sil_q88),
    .cfg_baud_div(cfg_baud_div), .cfg_msg_wm(cfg_msg_wm), .cfg_map_base_qw_iw(cfg_map_base_qw_iw),
    .stat_crc_err(stat_crc_err), .stat_lrc_err(stat_lrc_err), .stat_frame_to(stat_frame_to),
    .stat_tx_empty(stat_tx_empty), .stat_rx_avail(csr_rx_valid),
    .irq_w1c(irq_w1c),

    .scan_en(scan_en), .scan_retry_max(scan_retry_max), .scan_period_ms(scan_period_ms),
    .scan_cycles_done(scan_cycles_done), .scan_err_count(scan_err_count),
    .scan_idx(scan_idx), .scan_slave(scan_slave), .scan_func(scan_func),
    .scan_start_addr(scan_start_addr), .scan_qty(scan_qty), .scan_byte_count(scan_byte_count),
    .scan_wbase(scan_wbase), .scan_rbase(scan_rbase)
  );

  gpio_input  u_gpio_in(.clk(PCLK), .rst(rst), .gpio_i(GPIO_DI), .di_status(di_status));

  wire [31:0] do_wdata, do_wmask; wire do_we;
  gpio_output u_gpio_out(.clk(PCLK), .rst(rst), .do_wdata(do_wdata), .do_wmask(do_wmask), .do_we(do_we), .gpio_o(GPIO_DO));

  // UART bridge
  wire [7:0]  b_rx, b_tx;
  wire        b_rx_v, b_tx_v, b_tx_rdy;
  wire        f_start, f_end;
  /* verilator lint_off UNUSED */
  wire        f_to;
  wire        e_crc, e_lrc;
  /* verilator lint_on UNUSED */

  uart_bridge u_bridge(
    .clk(PCLK), .rst(rst),
    .baud_div(cfg_baud_div), .parity(cfg_parity), .stop2(cfg_stop2),
    .ascii_en(cfg_ascii_en), .rtu_sil_q88(cfg_rtu_sil_q88),
    .uart_rx_i(UART_RX), .uart_tx_o(UART_TX),
    .tx_data_i(b_tx), .tx_valid_i(b_tx_v), .tx_ready_o(b_tx_rdy),
    .rx_data_o(b_rx), .rx_valid_o(b_rx_v),
    .frame_start(f_start), .frame_end(f_end), .frame_timeout(f_to),
    .crc_err(e_crc), .lrc_err(e_lrc)
  );

  // Controller
  modbus_controller #(.REG_WORDS(REG_WORDS), .SCAN_MAX(SCAN_MAX)) u_ctrl(
    .clk(PCLK), .rst(rst),
    .cfg_mode_master(cfg_mode_master), .cfg_ascii_en(cfg_ascii_en), .cfg_map_base_qw_iw(cfg_map_base_qw_iw),
    .rx_b(b_rx), .rx_b_v(b_rx_v), .frame_start(f_start), .frame_end(f_end),
    .tx_b(b_tx), .tx_b_v(b_tx_v), .tx_b_rdy(b_tx_rdy),
    .di_status(di_status),
    .do_wdata(do_wdata), .do_wmask(do_wmask), .do_we(do_we),
    .stat_crc_err(stat_crc_err), .stat_lrc_err(stat_lrc_err), .stat_frame_to(stat_frame_to),
    .scan_en(scan_en), .scan_retry_max(scan_retry_max), .scan_period_ms(scan_period_ms),
    .scan_idx(scan_idx), .scan_slave(scan_slave), .scan_func(scan_func),
    .scan_start_addr(scan_start_addr), .scan_qty(scan_qty), .scan_byte_count(scan_byte_count),
    .scan_wbase(scan_wbase), .scan_rbase(scan_rbase),
    .scan_cycles_done(scan_cycles_done), .scan_err_count(scan_err_count)
  );

  // (Optional) expose csr MSG buffer to/from controller if you want host-driven frames
  assign csr_rx_data  = 8'h00; // Not used here; could be wired to controller RX FIFO
  assign csr_rx_valid = 1'b0;
  assign csr_tx_ready = 1'b1;  // Accepts writes (no-op)
endmodule
