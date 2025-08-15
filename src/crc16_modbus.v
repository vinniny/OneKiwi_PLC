// crc16_modbus.v - Modbus RTU CRC16 (poly 0xA001), init 0xFFFF, LSB-first
module crc16_modbus(
  input  wire       clk,
  input  wire       rst,
  input  wire       clr,
  input  wire [7:0] data_i,
  input  wire       valid_i,
  output reg  [15:0] crc_o
);
  reg [15:0] c;
  /* verilator lint_off BLKSEQ */
  function [15:0] step;
    input [15:0] cin;
    input [7:0]  d;
    integer i;
    reg [15:0] x;
    begin
      // Extend incoming byte to 16 bits before XOR to avoid width mismatch
      x = cin ^ {8'h00, d};
      for (i=0;i<8;i=i+1) begin
        if (x[0]) x = (x>>1) ^ 16'hA001;
        else      x = (x>>1);
      end
      step = x;
    end
  endfunction
  /* verilator lint_on BLKSEQ */

  always @(posedge clk) begin
    if (rst || clr) c <= 16'hFFFF;
    else if (valid_i) c <= step(c, data_i);
  end

  always @(*) crc_o = c;
endmodule
