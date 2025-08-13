// lrc_ascii.v - ASCII LRC = two's complement of sum of bytes
module lrc_ascii(
  input  wire       clk,
  input  wire       rst,
  input  wire       clr,
  input  wire [7:0] data_i,
  input  wire       valid_i,
  output reg  [7:0] lrc_o
);
  reg [7:0] sum;
  always @(posedge clk) begin
    if (rst || clr) sum<=8'h00;
    else if (valid_i) sum<=sum + data_i;
  end
  always @(*) lrc_o = (~sum) + 8'h01;
endmodule
