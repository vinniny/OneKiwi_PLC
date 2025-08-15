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
    apb_read(12'h000, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: DO reset %h", rddata); $finish; end
    apb_read(12'h004, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: DI reset %h", rddata); $finish; end
    apb_read(12'h008, rddata); if (rddata > 32'd16) begin $display("ERROR: TIMER reset %h", rddata); $finish; end
    apb_read(12'h00C, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: MSG reset %h", rddata); $finish; end
    apb_read(12'h010, rddata); if (rddata !== 32'h0001_0000) begin $display("ERROR: CFG0 reset %h", rddata); $finish; end
    apb_read(12'h014, rddata); if (rddata !== 32'h0080_0036) begin $display("ERROR: CFG1 reset %h", rddata); $finish; end
    apb_read(12'h018, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: MAP reset %h", rddata); $finish; end
    apb_read(12'h01C, rddata); if (rddata !== 32'h0000_0002) begin $display("ERROR: IRQ reset %h", rddata); $finish; end
    apb_read(12'h020, rddata); if (rddata !== 32'h0001_0014) begin $display("ERROR: SCAN_CTRL reset %h", rddata); $finish; end
    apb_read(12'h028, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: SCAN_IDX reset %h", rddata); $finish; end
    apb_read(12'h02C, rddata); if (rddata !== 32'h0001_0400) begin $display("ERROR: SCAN_ENTRY reset %h", rddata); $finish; end
    apb_read(12'h030, rddata); if (rddata !== 32'h0010_0010) begin $display("ERROR: SCAN_QTY reset %h", rddata); $finish; end
    apb_read(12'h034, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: SCAN_WBASE reset %h", rddata); $finish; end
    apb_read(12'h038, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: SCAN_RBASE reset %h", rddata); $finish; end

    // --- DO write/read tests ---
    apb_write(12'h000, 32'hDEADBEEF);
    apb_read(12'h000, rddata); if (rddata !== 32'hDEADBEEF) begin $display("ERROR: DO readback %h", rddata); $finish; end
    apb_write_masked(12'h000, 32'h1234_5678, 4'b0011);
    apb_read(12'h000, rddata); if (rddata !== 32'hDEAD_5678) begin $display("ERROR: DO partial %h", rddata); $finish; end

    // --- DI write should be ignored ---
    apb_write(12'h004, 32'hFFFF_FFFF);
    apb_read(12'h004, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: DI write ignored %h", rddata); $finish; end

    // --- Drive GPIO_DI and read DI ---
    GPIO_DI = 32'hA5A55A5A;
    repeat (2) @(posedge PCLK);
    apb_read(12'h004, rddata); if (rddata !== 32'hA5A55A5A) begin $display("ERROR: DI read %h", rddata); $finish; end

    // --- Timer write and verify increment ---
    apb_write(12'h008, 32'h0000_00F0);
    apb_read(12'h008, rddata); if (rddata < 32'h0000_00F0 || rddata > 32'h0000_00F3) begin $display("ERROR: Timer writeback %h", rddata); $finish; end
    repeat (10) @(posedge PCLK);
    apb_read(12'h008, rddata2); if (rddata2 <= rddata) begin $display("ERROR: Timer did not increment %h -> %h", rddata, rddata2); $finish; end

    // --- Configuration register writes ---
    apb_write(12'h010, 32'h0000_0105);
    apb_write(12'h014, 32'h0001_0020);
    apb_read(12'h010, rddata); if (rddata !== 32'h0000_0105) begin $display("ERROR: CFG0 mismatch %h", rddata); $finish; end
    apb_read(12'h014, rddata); if (rddata !== 32'h0001_0020) begin $display("ERROR: CFG1 mismatch %h", rddata); $finish; end

    // --- MAP base register ---
    apb_write(12'h018, 32'h0000_0044);
    apb_read(12'h018, rddata); if (rddata !== 32'h0000_0044) begin $display("ERROR: MAP write %h", rddata); $finish; end

    // --- IRQ W1C ---
    apb_write(12'h01C, 32'hFFFF_FFFF);
    apb_read(12'h01C, rddata); if (rddata !== 32'h0000_0002) begin $display("ERROR: IRQ W1C %h", rddata); $finish; end
    force dut.u_csr.stat_tx_empty = 1'b0;
    @(posedge PCLK);
    apb_write(12'h01C, 32'h0000_0002);
    apb_read(12'h01C, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: IRQ clear %h", rddata); $finish; end
    release dut.u_csr.stat_tx_empty;

    // --- Scan control/table registers ---
    apb_write(12'h020, 32'h0000_0101);
    apb_write(12'h028, 32'h0000_0001);
    apb_write(12'h02C, 32'h0100_0302);
    apb_write(12'h030, 32'h0004_0002);
    apb_write(12'h034, 32'h0000_0020);
    apb_write(12'h038, 32'h0000_0030);
    apb_read(12'h020, rddata); if (rddata !== 32'h0000_0101) begin $display("ERROR: SCAN_CTRL %h", rddata); $finish; end
    apb_read(12'h028, rddata); if (rddata !== 32'h0000_0001) begin $display("ERROR: SCAN_IDX %h", rddata); $finish; end
    apb_read(12'h02C, rddata); if (rddata !== 32'h0100_0302) begin $display("ERROR: SCAN_ENTRY %h", rddata); $finish; end
    apb_read(12'h030, rddata); if (rddata !== 32'h0004_0002) begin $display("ERROR: SCAN_QTY %h", rddata); $finish; end
    apb_read(12'h034, rddata); if (rddata !== 32'h0000_0020) begin $display("ERROR: SCAN_WBASE %h", rddata); $finish; end
    apb_read(12'h038, rddata); if (rddata !== 32'h0000_0030) begin $display("ERROR: SCAN_RBASE %h", rddata); $finish; end

    // --- Check PSLVERR stays low ---
    if (PSLVERR !== 1'b0) begin $display("ERROR: PSLVERR asserted"); $finish; end

    $display("All tests passed");
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
