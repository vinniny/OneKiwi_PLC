// uart_bridge.v - RTU/ASCII deframer + byte stream I/O
// - For RTU: detects frame by 3.5-char silence; passes raw bytes to controller.
// - For ASCII: detects ':' start and CRLF end; passes bytes (including ':'..'\n').
// Controller is responsible for CRC/LRC verification and TX frame formatting.
module uart_bridge #(
  /* verilator lint_off UNUSEDPARAM */
  parameter OVERSAMPLE = 16
  /* verilator lint_on UNUSEDPARAM */
)(
  input  wire        clk,
  input  wire        rst,

  input  wire [15:0] baud_div,
  input  wire [1:0]  parity,
  input  wire        stop2,
  input  wire        ascii_en,
  input  wire [15:0] rtu_sil_q88,  // Q8.8 "char units" threshold

  // physical
  input  wire        uart_rx_i,
  output wire        uart_tx_o,

  // TX byte stream (controller -> UART)
  input  wire [7:0]  tx_data_i,
  input  wire        tx_valid_i,
  output wire        tx_ready_o,

  // RX byte stream (UART -> controller)
  output reg  [7:0]  rx_data_o,
  output reg         rx_valid_o,

  // frame events
  output reg         frame_start,
  output reg         frame_end,
  output reg         frame_timeout,

  // sticky error outputs (the controller should W1C via CSR)
  output reg         crc_err,  // not computed here (wired from controller if desired)
  output reg         lrc_err
);
  // UART engines
  wire [7:0] rxd;
  wire       rxv;
  /* verilator lint_off UNUSED */
  wire       ferr, perr;
  /* verilator lint_on UNUSED */
  uart_rx u_rx(.clk(clk), .rst(rst), .rx_i(uart_rx_i),
               .baud_div(baud_div), .parity(parity), .stop2(stop2),
               .data_o(rxd), .valid_o(rxv), .framing_err(ferr), .parity_err(perr));
  reg  [7:0] txd; reg txv; wire txr;
  uart_tx u_tx(.clk(clk), .rst(rst), .baud_div(baud_div),
               .data_i(txd), .valid_i(txv), .ready_o(txr),
               .parity(parity), .stop2(stop2), .tx_o(uart_tx_o));
  assign tx_ready_o = txr;

  // transmit passthrough
  always @(posedge clk) begin
    if (rst) begin txd<=8'h00; txv<=1'b0; end
    else begin
      txv<=1'b0;
      if (tx_valid_i && txr) begin txd<=tx_data_i; txv<=1'b1; end
    end
  end

  // RTU silence detector
  // char_bits ≈ 10 + parity + stop2
  // Cast parity/stop flags to 4 bits to avoid width expansion warnings
  wire [3:0] char_bits = 4'd10 + (parity!=2'd0 ? 4'd1 : 4'd0) + (stop2 ? 4'd1 : 4'd0);
  reg  [23:0] sil_cnt;
  reg         in_frame;

  always @(posedge clk) begin
    if (rst) begin
      rx_data_o<=8'h00; rx_valid_o<=1'b0; frame_start<=1'b0; frame_end<=1'b0; frame_timeout<=1'b0;
      sil_cnt<=24'd0; in_frame<=1'b0; crc_err<=1'b0; lrc_err<=1'b0;
    end else begin
      rx_valid_o<=1'b0; frame_start<=1'b0; frame_end<=1'b0; frame_timeout<=1'b0;

      if (!ascii_en) begin
        // RTU
        if (rxv) begin
          if (!in_frame) begin in_frame<=1'b1; frame_start<=1'b1; sil_cnt<=24'd0; end
          sil_cnt<=24'd0;
          rx_data_o<=rxd; rx_valid_o<=1'b1;
        end else if (in_frame) begin
          if (sil_cnt < ({rtu_sil_q88,8'd0} * char_bits)) sil_cnt <= sil_cnt + {8'd0, baud_div};
          else begin in_frame<=1'b0; frame_end<=1'b1; end
        end
      end else begin
        // ASCII: ':' start, CRLF end
        if (rxv) begin
          if (!in_frame) begin
            if (rxd==8'h3A) begin in_frame<=1'b1; frame_start<=1'b1; rx_data_o<=rxd; rx_valid_o<=1'b1; end
          end else begin
            rx_data_o<=rxd; rx_valid_o<=1'b1;
            if (rxd==8'h0A) begin in_frame<=1'b0; frame_end<=1'b1; end // LF
          end
        end
      end
    end
  end
endmodule
