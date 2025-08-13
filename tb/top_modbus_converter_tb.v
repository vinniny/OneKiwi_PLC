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

    // --- Check default register values ---
    apb_read(12'h000, rddata); // DO
    if (rddata !== 32'h0000_0000) begin $display("ERROR: DO reset value %h", rddata); $finish; end
    apb_read(12'h004, rddata); // DI
    if (rddata !== 32'h0000_0000) begin $display("ERROR: DI reset value %h", rddata); $finish; end
    apb_read(12'h008, rddata); // TIMER should be small after reset
    if (rddata > 32'd16) begin $display("ERROR: TIMER reset value %h", rddata); $finish; end
    apb_read(12'h010, rddata); // CFG0
    if (rddata !== 32'h0001_0000) begin $display("ERROR: CFG0 reset value %h", rddata); $finish; end
    apb_read(12'h014, rddata); // CFG1
    if (rddata !== 32'h0080_0036) begin $display("ERROR: CFG1 reset value %h", rddata); $finish; end

    // --- Full write/read DO register ---
    apb_write(12'h000, 32'hDEADBEEF);
    apb_read(12'h000, rddata);
    if (rddata !== 32'hDEADBEEF) begin $display("ERROR: DO readback %h", rddata); $finish; end

    // --- Partial write to DO (upper half unchanged) ---
    apb_write_masked(12'h000, 32'h1234_5678, 4'b0011); // write lower 16b
    apb_read(12'h000, rddata);
    if (rddata !== 32'hDEAD_5678) begin $display("ERROR: DO partial write got %h", rddata); $finish; end

    // --- Attempt write to read-only DI register ---
    apb_write(12'h004, 32'hFFFF_FFFF);
    apb_read(12'h004, rddata);
    if (rddata !== 32'h0000_0000) begin $display("ERROR: DI should ignore writes %h", rddata); $finish; end

    // --- Drive GPIO_DI and read DI register ---
    GPIO_DI = 32'hA5A55A5A;
    repeat (2) @(posedge PCLK);
    apb_read(12'h004, rddata);
    if (rddata !== 32'hA5A55A5A) begin $display("ERROR: DI read %h", rddata); $finish; end

    // --- Write timer and verify increment ---
    apb_write(12'h008, 32'h0000_00F0);
    apb_read(12'h008, rddata);
    if (rddata < 32'h0000_00F0 || rddata > 32'h0000_00F3) begin $display("ERROR: Timer writeback %h", rddata); $finish; end
    repeat (10) @(posedge PCLK);
    apb_read(12'h008, rddata2);
    if (rddata2 <= rddata) begin $display("ERROR: Timer did not increment %h -> %h", rddata, rddata2); $finish; end

    // --- Write/read configuration registers ---
    apb_write(12'h010, 32'h0000_0105); // mode slave, parity even, stop2
    apb_write(12'h014, 32'h0001_0020); // baud_div=0x20, msg_wm=0x0001
    apb_read(12'h010, rddata);
    if (rddata !== 32'h0000_0105) begin $display("ERROR: CFG0 mismatch %h", rddata); $finish; end
    apb_read(12'h014, rddata);
    if (rddata !== 32'h0001_0020) begin $display("ERROR: CFG1 mismatch %h", rddata); $finish; end

    // --- Check PSLVERR stays low ---
    if (PSLVERR !== 1'b0) begin $display("ERROR: PSLVERR asserted"); $finish; end

    $display("All tests passed");

    // --- Write and read back DO register ---
    apb_write(12'h000, 32'hDEADBEEF);
    apb_read(12'h000, rddata);
    if (rddata !== 32'hDEADBEEF) begin
      $display("ERROR: DO readback %h", rddata);
      $finish;
    end

    // --- Drive GPIO_DI and read DI register ---
    GPIO_DI = 32'hA5A55A5A;
    repeat (5) @(posedge PCLK);
    apb_read(12'h004, rddata);
    if (rddata !== 32'hA5A55A5A) begin
      $display("ERROR: DI read %h", rddata);
      $finish;
    end

    // --- Check timer increment ---
    apb_read(12'h008, rddata);
    repeat (10) @(posedge PCLK);
    apb_read(12'h008, rddata2);
    if (rddata2 <= rddata) begin
      $display("ERROR: timer did not increment (%h -> %h)", rddata, rddata2);
      $finish;
    end

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

  // APB masked write task
  task apb_write_masked(input [11:0] addr, input [31:0] data, input [3:0] mask);
  begin
    @(posedge PCLK);
    PADDR  <= addr;
    PWDATA <= data;
    PWRITE <= 1'b1;
    PSEL   <= 1'b1;
    PSTRB  <= mask;
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

