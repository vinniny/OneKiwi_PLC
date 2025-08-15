// uart_tx.v - 16x oversampling UART transmitter, 8 data bits, optional parity, 1/2 stop
module uart_tx #(
  parameter [7:0] OVERSAMPLE = 8'd16
)(
  input  wire        clk,
  input  wire        rst,
  input  wire [15:0] baud_div,
  input  wire [7:0]  data_i,
  input  wire        valid_i,
  output reg         ready_o,
  input  wire [1:0]  parity,  // 0=None,1=Even,2=Odd
  input  wire        stop2,
  output reg         tx_o
);
  localparam [2:0] S_IDLE=3'd0, S_START=3'd1, S_DATA=3'd2, S_PAR=3'd3, S_STOP=3'd4;
  reg [2:0]  st;
  reg [15:0] div;
  reg        tick;
  reg [7:0]  os;
  reg [2:0]  bitn;
  reg [7:0]  sh;
  reg        par_acc;

  always @(posedge clk) begin
    if (rst) begin
      div <= 16'd0;
      tick <= 1'b0;
    end else begin
      tick <= 1'b0;
      if (div == 16'd0) begin
        div <= baud_div;
        tick <= 1'b1;
      end else begin
        div <= div - 16'd1;
      end
    end
  end

  always @(posedge clk) begin
    if (rst) begin
      st <= S_IDLE;
      tx_o <= 1'b1;
      ready_o <= 1'b1;
      os <= 8'd0;
      bitn <= 3'd0;
      sh <= 8'd0;
      par_acc <= 1'b0;
    end else if (tick) begin
      case (st)
        S_IDLE: begin
          tx_o <= 1'b1;
          ready_o <= 1'b1;
          if (valid_i) begin
            ready_o <= 1'b0;
            sh <= data_i;
            par_acc <= ^data_i;
            st <= S_START;
            os <= OVERSAMPLE - 8'd1;
            tx_o <= 1'b0;
          end
        end
        S_START: begin
          if (os == 8'd0) begin
            st <= S_DATA;
            os <= OVERSAMPLE - 8'd1;
            bitn <= 3'd0;
          end else begin
            os <= os - 8'd1;
          end
        end
        S_DATA: begin
          tx_o <= sh[0];
          if (os == 8'd0) begin
            sh <= {1'b0, sh[7:1]};
            os <= OVERSAMPLE - 8'd1;
            if (bitn == 3'd7) begin
              if (parity == 2'd0)
                st <= S_STOP;
              else
                st <= S_PAR;
            end
            bitn <= bitn + 3'd1;
          end else begin
            os <= os - 8'd1;
          end
        end
        S_PAR: begin
          tx_o <= (parity == 2'd1) ? par_acc : ~par_acc;
          if (os == 8'd0) begin
            st <= S_STOP;
            os <= OVERSAMPLE - 8'd1;
          end else begin
            os <= os - 8'd1;
          end
        end
        S_STOP: begin
          tx_o <= 1'b1;
          if (os == 8'd0) begin
            if (stop2) begin
              os <= OVERSAMPLE - 8'd1;
              st <= S_IDLE;
              ready_o <= 1'b1;
            end else begin
              st <= S_IDLE;
              ready_o <= 1'b1;
            end
          end else begin
            os <= os - 8'd1;
          end
        end
        default: begin
          st <= S_IDLE;
          tx_o <= 1'b1;
          ready_o <= 1'b1;
        end
      endcase
    end
  end
endmodule
