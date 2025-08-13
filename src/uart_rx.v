// uart_rx.v - 16x oversampling UART receiver, 8 data bits, optional parity, 1/2 stop
module uart_rx #(
  parameter OVERSAMPLE = 16
)(
  input  wire        clk,
  input  wire        rst,
  input  wire        rx_i,
  input  wire [15:0] baud_div,    // generates 16x sample tick
  input  wire [1:0]  parity,      // 0=None,1=Even,2=Odd
  input  wire        stop2,       // 0=1 stop, 1=2 stops
  output reg  [7:0]  data_o,
  output reg         valid_o,
  output reg         framing_err,
  output reg         parity_err
);
  localparam [2:0] S_IDLE=3'd0, S_START=3'd1, S_DATA=3'd2, S_PAR=3'd3, S_STOP=3'd4;
  reg [2:0]  st;
  reg [15:0] div;
  reg        tick;
  reg [3:0]  os;
  reg [2:0]  bitn;
  reg [7:0]  sh;
  reg        par_acc;

  always @(posedge clk) begin
    if (rst) begin div<=16'd0; tick<=1'b0; end
    else begin
      tick<=1'b0;
      if (div==16'd0) begin div<=baud_div; tick<=1'b1; end
      else div<=div-16'd1;
    end
  end

  always @(posedge clk) begin
    if (rst) begin
      st<=S_IDLE; os<=0; bitn<=0; sh<=0; par_acc<=0;
      valid_o<=0; framing_err<=0; parity_err<=0;
    end else begin
      valid_o<=1'b0;
      if (tick) begin
        case (st)
          S_IDLE: begin
            framing_err<=1'b0; parity_err<=1'b0; par_acc<=1'b0;
            if (~rx_i) begin st<=S_START; os<=OVERSAMPLE/2; end
          end
          S_START: begin
            if (os==0) begin
              if (~rx_i) begin st<=S_DATA; os<=OVERSAMPLE-1; bitn<=3'd0; end
              else st<=S_IDLE;
            end else os<=os-1;
          end
          S_DATA: begin
            if (os==0) begin
              sh <= {rx_i, sh[7:1]};
              par_acc <= par_acc ^ rx_i;
              os<=OVERSAMPLE-1;
              if (bitn==3'd7) begin
                if (parity==2'd0) st<=S_STOP;
                else st<=S_PAR;
              end
              bitn<=bitn+3'd1;
            end else os<=os-1;
          end
          S_PAR: begin
            if (os==0) begin
              if (parity==2'd1) parity_err <= (par_acc != rx_i);
              else if (parity==2'd2) parity_err <= (par_acc == rx_i);
              st<=S_STOP; os<=OVERSAMPLE-1;
            end else os<=os-1;
          end
          S_STOP: begin
            if (os==0) begin
              if (~rx_i) framing_err<=1'b1;
              data_o<=sh; valid_o<=1'b1;
              if (stop2) begin os<=OVERSAMPLE-1; st<=S_IDLE; end
              else st<=S_IDLE;
            end else os<=os-1;
          end
        endcase
      end
    end
  end
endmodule
