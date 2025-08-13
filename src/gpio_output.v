// gpio_output.v - registered DO with write-mask
module gpio_output #(
  parameter WIDTH=32
)(
  input  wire             clk,
  input  wire             rst,
  input  wire [WIDTH-1:0] do_wdata,
  input  wire [WIDTH-1:0] do_wmask,
  input  wire             do_we,     // strobe
  output reg  [WIDTH-1:0] gpio_o
);
  always @(posedge clk) begin
    if (rst) gpio_o<=0;
    else if (do_we) gpio_o <= (gpio_o & ~do_wmask) | (do_wdata & do_wmask);
  end
endmodule
