`timescale 1ns/1ps

// Top-level testbench that exercises individual modules then the full system
module top_modbus_converter_tb;
  // Clock and reset
  reg PCLK;
  reg PRESETn;

  // APB3 interface
  reg [11:0] PADDR;
  reg        PSEL;
  reg        PENABLE;
  reg        PWRITE;
  reg [31:0] PWDATA;
  reg [3:0]  PSTRB;
  wire [31:0] PRDATA;
  wire       PREADY;
  wire       PSLVERR;

  // UART pins for DUT
  reg  UART_RX;
  wire UART_TX;

  // GPIOs
  reg  [31:0] GPIO_DI;
  wire [31:0] GPIO_DO;

  // Debug signals to observe Modbus controller behavior
  wire [31:0] dbg_do_wdata;
  wire [31:0] dbg_do_wmask;
  wire        dbg_do_we;
  wire        dbg_crc_err;

  // Instantiate top-level DUT
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

`ifdef TB_EXPORT_INTERNALS
  assign dbg_do_wdata = dut.do_wdata;
  assign dbg_do_wmask = dut.do_wmask;
  assign dbg_do_we    = dut.do_we;
  assign dbg_crc_err  = dut.stat_crc_err;
`else
  assign dbg_do_wdata = 32'h0;
  assign dbg_do_wmask = 32'h0;
  assign dbg_do_we    = 1'b0;
  assign dbg_crc_err  = 1'b0;
`endif

`ifdef TB_EXPORT_INTERNALS
  // Guard against simultaneous frame start/end
  always @(posedge PCLK) if (PRESETn) begin
    if (dut.f_start && dut.f_end) begin
      $display("ERROR: frame_start and frame_end asserted together");
      $finish;
    end
  end

  // Ensure DO write pulse is one cycle with stable data
  reg        do_we_q;
  reg [31:0] dbg_do_wmask_q;
  reg [31:0] dbg_do_wdata_q;
  always @(posedge PCLK) begin
    do_we_q        <= dbg_do_we;
    dbg_do_wmask_q <= dbg_do_wmask;
    dbg_do_wdata_q <= dbg_do_wdata;
    if (dbg_do_we) begin
      if (dbg_do_wmask !== dbg_do_wmask_q) begin
        $display("ERROR: do_wmask changed while do_we asserted");
        $finish;
      end
      if (dbg_do_wdata !== dbg_do_wdata_q) begin
        $display("ERROR: do_wdata changed while do_we asserted");
        $finish;
      end
    end
    if (do_we_q && dbg_do_we) begin
      $display("ERROR: do_we wider than 1 cycle");
      $finish;
    end
  end

  // RX length bound
  always @(posedge PCLK) if (PRESETn) begin
    if (dut.u_ctrl.rx_len > 8'd255) begin
      $display("ERROR: rx_len overflow");
      $finish;
    end
  end
`endif

  // Generate 100 MHz clock
  initial PCLK = 0;
  always #5 PCLK = ~PCLK;

  // Create a synchronous reset for standalone module tests
  reg [1:0] rst_ff;
  wire      rst;
  always @(posedge PCLK or negedge PRESETn) begin
    if (!PRESETn)
      rst_ff <= 2'b00;
    else
      rst_ff <= {rst_ff[0],1'b1};
  end
  assign rst = ~rst_ff[1];

  // Standalone UART bridge instance for unit test
  reg        tb_uart_rx;
  wire       tb_uart_tx;
  wire [7:0] tb_b_rx;
  wire       tb_b_rx_v;
  wire       tb_f_start, tb_f_end;
  wire       tb_tx_ready;
  wire       tb_f_timeout;
  wire       tb_crc_err, tb_lrc_err;

  uart_bridge u_bridge_test(
    .clk(PCLK),
    .rst(rst),
    .baud_div(16'd54),
    .parity(2'd0),
    .stop2(1'b0),
    .ascii_en(1'b0),
    // use 3.5 character clamp for standalone test
    .rtu_sil_q88(16'd0),
    .uart_rx_i(tb_uart_rx),
    .uart_tx_o(tb_uart_tx),
    .tx_data_i(8'h00),
    .tx_valid_i(1'b0),
    .tx_ready_o(tb_tx_ready),
    .rx_data_o(tb_b_rx),
    .rx_valid_o(tb_b_rx_v),
    .frame_start(tb_f_start),
    .frame_end(tb_f_end),
    .frame_timeout(tb_f_timeout),
    .crc_err(tb_crc_err),
    .lrc_err(tb_lrc_err)
  );

  // Shared test variables
  reg [31:0] rddata, rddata2;
  integer    bit_cycles, half_bit_cycles;
  integer    i;
  reg [7:0]  b0, b1, b2, b3, b4, b5, b6, b7;
  reg [7:0]  tb_buf [0:255];

  // Main test sequence
  initial begin
    // Default stimulus
    PRESETn = 0;
    PADDR   = 0;
    PSEL    = 0;
    PENABLE = 0;
    PWRITE  = 0;
    PWDATA  = 0;
    PSTRB   = 4'hF;
    UART_RX = 1'b1;
    GPIO_DI = 32'h0;
    tb_uart_rx = 1'b1;

    // Apply reset
    repeat (5) @(posedge PCLK);
    PRESETn = 1'b1;
    repeat (5) @(posedge PCLK);

    // Exercise individual modules
    test_csr_and_gpio();
    test_uart_bridge_unit();

    // Full system Modbus stimulus
    test_full_system_modbus();
    test_ascii_modbus();
    test_bad_crc();

    // Ensure no bus errors
    if (PSLVERR !== 1'b0) begin
      $display("ERROR: PSLVERR asserted");
      $finish;
    end

    $display("All tests completed successfully");
    $finish;
  end

  // ----------------------
  // Unit test tasks
  // ----------------------

  // Test CSR block and GPIO paths via APB
  task test_csr_and_gpio;
  begin
    $display("Starting test_csr_and_gpio");
    // --- Check default register values ---
    apb_read(12'h000, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: DO reset %h", rddata); $finish; end
    apb_read(12'h004, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: DI reset %h", rddata); $finish; end
    apb_read(12'h008, rddata); if (rddata > 32'd16) begin $display("ERROR: TIMER reset %h", rddata); $finish; end
    apb_read(12'h00C, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: MSG reset %h", rddata); $finish; end
    apb_read(12'h010, rddata); if (rddata !== 32'h0001_0000) begin $display("ERROR: CFG0 reset %h", rddata); $finish; end
    apb_read(12'h014, rddata); if (rddata !== 32'h0080_0036) begin $display("ERROR: CFG1 reset %h", rddata); $finish; end
    bit_cycles      = (rddata[15:0] + 1) * 16;  // Account for divider reload
    half_bit_cycles = bit_cycles / 2;
    $display("Calculated bit_cycles: %0d (baud_div=%0d)", bit_cycles, rddata[15:0]);
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
    repeat (3) @(posedge PCLK);
    apb_read(12'h004, rddata); if (rddata !== 32'hA5A55A5A) begin $display("ERROR: DI read %h", rddata); $finish; end

    // --- Timer write and verify increment ---
    apb_write(12'h008, 32'h0000_00F0);
    apb_read(12'h008, rddata); if (rddata < 32'h0000_00F0 || rddata > 32'h0000_00F3) begin $display("ERROR: Timer writeback %h", rddata); $finish; end
    repeat (10) @(posedge PCLK);
    apb_read(12'h008, rddata2); if (rddata2 <= rddata) begin $display("ERROR: Timer did not increment %h -> %h", rddata, rddata2); $finish; end

    // --- Configuration register writes ---
    apb_write(12'h010, 32'h0000_0000);
    apb_write(12'h014, 32'h0080_0036);
    apb_read(12'h010, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: CFG0 mismatch %h", rddata); $finish; end
    apb_read(12'h014, rddata); if (rddata !== 32'h0080_0036) begin $display("ERROR: CFG1 mismatch %h", rddata); $finish; end
    bit_cycles      = (rddata[15:0] + 1) * 16;  // Match UART oversample rate
    half_bit_cycles = bit_cycles / 2;
    $display("Updated bit_cycles: %0d (baud_div=%0d)", bit_cycles, rddata[15:0]);

    // --- MAP base register ---
    apb_write(12'h018, 32'h0000_0044);
    apb_read(12'h018, rddata); if (rddata !== 32'h0000_0044) begin $display("ERROR: MAP write %h", rddata); $finish; end

    // --- IRQ W1C ---
    apb_write(12'h01C, 32'hFFFF_FFFF);
    apb_read(12'h01C, rddata); if (rddata !== 32'h0000_0002) begin $display("ERROR: IRQ W1C %h", rddata); $finish; end
    force dut.stat_tx_empty = 1'b0;
    @(posedge PCLK);
    apb_write(12'h01C, 32'h0000_0002);
    apb_read(12'h01C, rddata); if (rddata !== 32'h0000_0000) begin $display("ERROR: IRQ clear %h", rddata); $finish; end
    release dut.stat_tx_empty;

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
    apb_write(12'h020, 32'h0000_0000);
    $display("test_csr_and_gpio passed");
  end
  endtask

  // Simple stand-alone UART bridge receive test
  task test_uart_bridge_unit;
  begin
    $display("Starting test_uart_bridge_unit");
    if (!tb_tx_ready) begin $display("ERROR: uart_bridge not ready"); $finish; end
    $display("Sending test bytes with bit_cycles=%0d", bit_cycles);
    // Allow initial idle before starting frame
    for (i=0; i<bit_cycles*5; i=i+1) @(posedge PCLK);
    uart_send_byte_unit(8'h55);
    uart_send_byte_unit(8'hAA);
    $display("Bytes sent; waiting for frame end");
    tb_uart_rx = 1'b1;
    i = 0;
    while (i < bit_cycles*400 && !tb_f_end) begin
      @(posedge PCLK); i = i + 1;
    end
    if (!tb_f_end) begin $display("ERROR: tb_f_end not asserted after %0d clocks", i); $finish; end
    $display("tb_f_end asserted after %0d idle clocks", i);
    $display("test_uart_bridge_unit passed");
  end
  endtask

  // Full system Modbus frame to drive GPIO
  task test_full_system_modbus;
  reg saw_we;
  begin
    $display("Starting test_full_system_modbus");
    // Switch to slave mode with short silent interval
    apb_write(12'h010, 32'h0100_0000);

    // Clear DO register so Modbus write has known starting point
    apb_write(12'h000, 32'h0000_0000);

    // Ensure idle time before frame start (3.5 chars)
    for (i=0; i<bit_cycles*5; i=i+1) @(posedge PCLK);

    // Send Modbus write-single-coil frame
    uart_send_byte_dut(8'h01); // addr
    uart_send_byte_dut(8'h05); // write single coil
    uart_send_byte_dut(8'h00); // coil high
    uart_send_byte_dut(8'h00); // coil low
    uart_send_byte_dut(8'hFF); // data high
    uart_send_byte_dut(8'h00); // data low
    uart_send_byte_dut(8'h8C); // CRC lo
    uart_send_byte_dut(8'h3A); // CRC hi

    // Add idle time to trigger frame_end
    UART_RX = 1'b1;
    for (i=0; i<bit_cycles*40; i=i+1) @(posedge PCLK);

    // Wait for controller write strobe and allow response to complete
    wait (dbg_do_we);
    $display("do_we asserted: mask=%h data=%h", dbg_do_wmask, dbg_do_wdata);
    for (i=0; i<bit_cycles*40; i=i+1) @(posedge PCLK);

    // Read back DO register and check GPIO
    apb_read(12'h000, rddata);
    $display("DO register readback = %h", rddata);
    if (rddata[0] !== 1'b1)
      $display("ERROR: DO register bit0 not set");
    if (GPIO_DO[0] !== 1'b1)
      $display("ERROR: GPIO_DO not updated by Modbus write");
    $display("test_full_system_modbus passed");
  end
  endtask

  // ASCII helpers
  function [7:0] hex_char;
    input [3:0] nib;
    begin
      if (nib < 4'd10)
        hex_char = "0" + nib[3:0];
      else
        hex_char = "A" + (nib[3:0] - 4'd10);
    end
  endfunction

  task uart_send_hex_byte(input [7:0] data);
  begin
    uart_send_byte_dut(hex_char(data[7:4]));
    uart_send_byte_dut(hex_char(data[3:0]));
  end
  endtask

  task send_ascii_frame;
    input integer n;
    integer i; reg [7:0] lrc;
    begin
      lrc = 8'h00;
      for (i=0; i<n; i=i+1) lrc = lrc + tb_buf[i];
      lrc = (~lrc) + 8'h01;
      uart_send_byte_dut(":");
      for (i=0;i<n;i=i+1) uart_send_hex_byte(tb_buf[i]);
      uart_send_hex_byte(lrc);
      uart_send_byte_dut(8'h0D);
      uart_send_byte_dut(8'h0A);
    end
  endtask

  task test_ascii_modbus;
  reg [7:0] exp[0:255];
  reg [7:0] rx;
  reg [7:0] lrc;
  integer idx;
  begin
    $display("Starting test_ascii_modbus");
    // Enable ASCII mode
    apb_write(12'h010, 32'h0100_0100);
    apb_write(12'h000, 32'h0000_0000);
    for (i=0; i<bit_cycles*5; i=i+1) @(posedge PCLK);

    // Build and send FC05 frame in ASCII
    tb_buf[0]=8'h01; tb_buf[1]=8'h05; tb_buf[2]=8'h00; tb_buf[3]=8'h00; tb_buf[4]=8'hFF; tb_buf[5]=8'h00;
    send_ascii_frame(6);
    UART_RX = 1'b1;

    // Expected echo
    lrc = 8'h00;
    for (idx=0; idx<6; idx=idx+1) lrc = lrc + tb_buf[idx];
    lrc = (~lrc) + 8'h01;
    exp[0] = ":";
    for (idx=0; idx<6; idx=idx+1) begin
      exp[1+idx*2] = hex_char(tb_buf[idx][7:4]);
      exp[2+idx*2] = hex_char(tb_buf[idx][3:0]);
    end
    exp[13] = hex_char(lrc[7:4]);
    exp[14] = hex_char(lrc[3:0]);
    exp[15] = 8'h0D;
    exp[16] = 8'h0A;

    wait (dbg_do_we);
    for (i=0; i<bit_cycles*40; i=i+1) @(posedge PCLK);

    for (idx=0; idx<17; idx=idx+1) begin
      uart_recv_byte_dut(rx);
      if (rx !== exp[idx]) begin
        $display("ERROR: ASCII echo mismatch at %0d exp=%02x got=%02x", idx, exp[idx], rx);
        $finish;
      end
    end

    apb_read(12'h000, rddata);
    if (rddata[0] !== 1'b1)
      $display("ERROR: DO register bit0 not set (ASCII)");

    // Return to RTU mode
    apb_write(12'h010, 32'h0100_0000);
    $display("test_ascii_modbus passed");
  end
  endtask

  task test_bad_crc;
  reg saw_we;
  integer j;
  begin
    $display("Starting test_bad_crc");
    apb_write(12'h010, 32'h0100_0000);
    apb_write(12'h000, 32'h0000_0000);
    for (i=0; i<bit_cycles*5; i=i+1) @(posedge PCLK);

    // Frame with incorrect CRC
    uart_send_byte_dut(8'h01);
    uart_send_byte_dut(8'h05);
    uart_send_byte_dut(8'h00);
    uart_send_byte_dut(8'h00);
    uart_send_byte_dut(8'hFF);
    uart_send_byte_dut(8'h00);
    uart_send_byte_dut(8'h8D); // corrupt CRC lo
    uart_send_byte_dut(8'h3A); // CRC hi
    UART_RX = 1'b1;

    saw_we = 1'b0;
    for (j=0; j<bit_cycles*40; j=j+1) begin
      @(posedge PCLK);
      if (dbg_do_we) saw_we = 1'b1;
      if (UART_TX == 1'b0) begin
        $display("ERROR: unexpected TX on bad CRC");
        $finish;
      end
    end
    if (saw_we) begin
      $display("ERROR: do_we asserted on bad CRC");
      $finish;
    end

    apb_read(12'h000, rddata);
    if (rddata[0] !== 1'b0)
      $display("ERROR: DO register changed on bad CRC");
    $display("test_bad_crc passed");
  end
  endtask

  // ----------------------
  // Helper tasks
  // ----------------------

  task apb_write(input [11:0] addr, input [31:0] data);
  begin
    PADDR  = addr;
    PWDATA = data;
    PWRITE = 1'b1;
    PSEL   = 1'b1;
    PSTRB  = 4'hF;
    @(posedge PCLK);
    PENABLE = 1'b1;
    while (!PREADY) @(posedge PCLK);
    @(posedge PCLK);
    PSEL   = 1'b0;
    PENABLE = 1'b0;
    PWRITE = 1'b0;
  end
  endtask

  task apb_write_masked(input [11:0] addr, input [31:0] data, input [3:0] mask);
  begin
    PADDR  = addr;
    PWDATA = data;
    PWRITE = 1'b1;
    PSEL   = 1'b1;
    PSTRB  = mask;
    @(posedge PCLK);
    PENABLE = 1'b1;
    while (!PREADY) @(posedge PCLK);
    @(posedge PCLK);
    PSEL   = 1'b0;
    PENABLE = 1'b0;
    PWRITE = 1'b0;
  end
  endtask

  task apb_read(input [11:0] addr, output [31:0] data);
  begin
    PADDR  = addr;
    PWRITE = 1'b0;
    PSEL   = 1'b1;
    PSTRB  = 4'h0;
    @(posedge PCLK);
    PENABLE = 1'b1;
    while (!PREADY) @(posedge PCLK);
    data = PRDATA;
    @(posedge PCLK);
    PSEL   = 1'b0;
    PENABLE = 1'b0;
  end
  endtask

  // UART send byte to DUT
  task uart_send_byte_dut(input [7:0] data);
    integer bitn, cyc;
    begin
      UART_RX = 1'b0;
      for (cyc=0; cyc<bit_cycles; cyc=cyc+1) @(posedge PCLK);
      for (bitn=0; bitn<8; bitn=bitn+1) begin
        UART_RX = data[bitn];
        for (cyc=0; cyc<bit_cycles; cyc=cyc+1) @(posedge PCLK);
      end
      UART_RX = 1'b1;
      for (cyc=0; cyc<bit_cycles; cyc=cyc+1) @(posedge PCLK);
    end
  endtask

  // UART receive byte from DUT
  task uart_recv_byte_dut(output [7:0] data);
    integer bitn, cyc;
    integer timeout;
    begin
      // Wait for start bit but bail out if it never arrives
      timeout = bit_cycles*1000;
      while (UART_TX == 1'b1 && timeout > 0) begin
        @(posedge PCLK);
        timeout = timeout - 1;
      end
      if (UART_TX == 1'b1) begin
        $display("ERROR: uart_recv_byte_dut timeout waiting for start bit");
        data = 8'h00;
        $finish;
      end else begin
        for (cyc=0; cyc<bit_cycles+half_bit_cycles; cyc=cyc+1) @(posedge PCLK);
        data[0] = UART_TX;
        for (bitn=1; bitn<8; bitn=bitn+1) begin
          for (cyc=0; cyc<bit_cycles; cyc=cyc+1) @(posedge PCLK);
          data[bitn] = UART_TX;
        end
        for (cyc=0; cyc<bit_cycles; cyc=cyc+1) @(posedge PCLK);
      end
    end
  endtask

  // UART send byte to standalone bridge
  task uart_send_byte_unit(input [7:0] data);
    integer bitn, cyc;
    begin
      tb_uart_rx = 1'b0;
      for (cyc=0; cyc<bit_cycles; cyc=cyc+1) @(posedge PCLK);
      for (bitn=0; bitn<8; bitn=bitn+1) begin
        tb_uart_rx = data[bitn];
        for (cyc=0; cyc<bit_cycles; cyc=cyc+1) @(posedge PCLK);
      end
      tb_uart_rx = 1'b1;
      for (cyc=0; cyc<bit_cycles; cyc=cyc+1) @(posedge PCLK);
    end
  endtask

endmodule

