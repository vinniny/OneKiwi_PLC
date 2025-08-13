// gpio_input.v - double-flop synchronizer + register capture
module gpio_input #(
  parameter WIDTH=32
)(
  input  wire             clk,
  input  wire             rst,
  input  wire [WIDTH-1:0] gpio_i,
  output reg  [WIDTH-1:0] di_status
);
  reg [WIDTH-1:0] s1, s2;
  always @(posedge clk) begin
    if (rst) begin s1<=0; s2<=0; di_status<=0; end
    else begin s1<=gpio_i; s2<=s1; di_status<=s2; end
  end
endmodule
