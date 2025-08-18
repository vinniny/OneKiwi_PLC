`timescale 1ns / 1ps

module uart_loopback_tb();

	parameter DATA_WIDTH = 256;
	parameter MSB_FIRST = 0;
	
	reg Clk;
	reg Rst_n;
	wire uart_rx;
	wire [2:0]led;
	wire uart_tx;
	
    reg [DATA_WIDTH - 1 : 0]data;
    reg send_en;
    wire uart_tx_test;
    wire Tx_Done_test;
    wire uart_state_test;
	
    assign uart_rx = uart_tx_test;
    
    //模拟串口的输入数据
	uart_data_tx
	#(
		.DATA_WIDTH(DATA_WIDTH),
		.MSB_FIRST(MSB_FIRST)
	)
	uart_data_tx_tset(
		.Clk(Clk),
		.Rst_n(Rst_n),
		.data(data),
		.send_en(send_en),   
		.Baud_Set(3'd4),  
		.uart_tx(uart_tx_test),  
		.Tx_Done(Tx_Done_test),   
		.uart_state(uart_state_test)
	);
    
    uart_loopback 
    #(
		.DATA_WIDTH(DATA_WIDTH),
		.MSB_FIRST(MSB_FIRST)
	)uart_loopback(
        .Clk(Clk),
        .Rst_n(Rst_n),
        .uart_rx(uart_rx),
        
        .led(led),
        .uart_tx(uart_tx)
     );
 
 	initial Clk = 1;
	always #10 Clk = !Clk;
	
	initial begin
		Rst_n = 0;
		data = 0;
		send_en = 0;
		#201;
		Rst_n = 1;
		#2000;
		data = 256'h890abcdef12312345abcdef674567890cba0987654fed365432121fedcba0987;
		send_en = 1;
		#20;
		send_en = 0;
		#20;
		@(posedge Tx_Done_test);
//        #1;
		#1000000;
		data = 256'hcba0987654fed365432121fedcba0987ba09876fe321dc54b321dc6fe54a0978;
		send_en = 1;
		#20;
		send_en = 0;
		#20;
		@(posedge Tx_Done_test);
//		#1;	
		#1000000;
		data = 256'h890abcdef123123454fed365432121fedcba09875abcdef674567890cba09876;
		send_en = 1;
		#20;
		send_en = 0;
		#20;
		@(posedge Tx_Done_test);
		#1;
		#4000000;
		$stop;		
	end
endmodule
