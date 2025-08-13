`timescale 1ns/1ps

module top_modbus_converter_tb;
  // Clock and reset
  reg PCLK;
  reg PRESETn;

  // APB3 interface
  reg [11:0] PADDR;
  reg PSEL;
  reg PENABLE;
  reg PWRITE;
  reg [31:0] PWDATA;
  reg [3:0]  PSTRB;
  wire [31:0] PRDATA;
  wire PREADY;
  wire PSLVERR;

  // UART pins
  reg  UART_RX;
  wire UART_TX;

  // GPIOs
  reg  [31:0] GPIO_DI;
  wire [31:0] GPIO_DO;

  // Instantiate DUT
  top_modbus_converter dut(
    .PCLK(PCLK),
    .PRESETn(PRESETn),
    .PADDR(PADDR),
    .PSEL(PSEL),
    .PENABLE(PENABLE),
    .PWRITE(PWRITE),
    .PWDATA(PWDATA),
    .PSTRB(PSTRB),
    .PRDATA(PRDATA),
    .PREADY(PREADY),
    .PSLVERR(PSLVERR),
    .UART_RX(UART_RX),
    .UART_TX(UART_TX),
    .GPIO_DI(GPIO_DI),
    .GPIO_DO(GPIO_DO)
  );

  // Clock generation (100 MHz)
  initial PCLK = 0;
  always #5 PCLK = ~PCLK;

  // Test variables
  reg [31:0] rddata, rddata2;

  // Test sequence
  initial begin
    // Initialize inputs
    PRESETn = 0;
    PADDR   = 0;
    PSEL    = 0;
    PENABLE = 0;
    PWRITE  = 0;
    PWDATA  = 0;
    PSTRB   = 4'hF;
    UART_RX = 1'b1; // idle
    GPIO_DI = 32'd0;

    // Apply reset
    repeat (5) @(posedge PCLK);
    PRESETn = 1;
    repeat (5) @(posedge PCLK);

    // --- Write and read back DO register ---
    apb_write(12'h000, 32'hDEADBEEF);
    apb_read(12'h000, rddata);
    if (rddata !== 32'hDEADBEEF)
      $display("ERROR: DO readback %h", rddata);
    else
      $display("DO readback OK: %h", rddata);

    // --- Drive GPIO_DI and read DI register ---
    GPIO_DI = 32'hA5A55A5A;
    repeat (5) @(posedge PCLK);
    apb_read(12'h004, rddata);
    if (rddata !== 32'hA5A55A5A)
      $display("ERROR: DI read %h", rddata);
    else
      $display("DI read OK: %h", rddata);

    // --- Check timer increment ---
    apb_read(12'h008, rddata);
    repeat (10) @(posedge PCLK);
    apb_read(12'h008, rddata2);
    if (rddata2 <= rddata)
      $display("ERROR: timer did not increment (%h -> %h)", rddata, rddata2);
    else
      $display("Timer increment OK: %h -> %h", rddata, rddata2);

    $display("Testbench completed");
    #20 $finish;
  end

  // APB write task
  task apb_write(input [11:0] addr, input [31:0] data);
  begin
    @(posedge PCLK);
    PADDR  <= addr;
    PWDATA <= data;
    PWRITE <= 1'b1;
    PSEL   <= 1'b1;
    PSTRB  <= 4'hF;
    @(posedge PCLK);
    PENABLE <= 1'b1;
    while (!PREADY) @(posedge PCLK);
    PSEL   <= 1'b0;
    PENABLE<= 1'b0;
    PWRITE <= 1'b0;
  end
  endtask

  // APB read task
  task apb_read(input [11:0] addr, output [31:0] data);
  begin
    @(posedge PCLK);
    PADDR  <= addr;
    PWRITE <= 1'b0;
    PSEL   <= 1'b1;
    PSTRB  <= 4'h0;
    @(posedge PCLK);
    PENABLE <= 1'b1;
    while (!PREADY) @(posedge PCLK);
    data = PRDATA;
    PSEL   <= 1'b0;
    PENABLE<= 1'b0;
  end
  endtask
endmodule

