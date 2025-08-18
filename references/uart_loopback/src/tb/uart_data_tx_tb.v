module uart_data_tx_tb();

	parameter DATA_WIDTH = 32;
	parameter MSB_FIRST = 0;

	reg Clk;
	reg Rst_n;
	
	reg [DATA_WIDTH - 1 : 0]data;
	reg send_en;
	wire uart_tx;
	wire Tx_Done;
	wire uart_state;

	uart_data_tx
	#(
		.DATA_WIDTH(DATA_WIDTH),
		.MSB_FIRST(MSB_FIRST)
	)
	uart_data_tx(
		.Clk(Clk),
		.Rst_n(Rst_n),
		.data(data),
		.send_en(send_en),   
		.Baud_Set(3'd4),  
		.uart_tx(uart_tx),  
		.Tx_Done(Tx_Done),   
		.uart_state(uart_state)
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
		data = 32'h01234567;
		send_en = 1;
		#20;
		send_en = 0;
		#20;
		@(posedge Tx_Done);
		#1;
		data = 32'h12345678;
		send_en = 1;
		#20;
		send_en = 0;
		#20;
		@(posedge Tx_Done);
		#1;	
		data = 32'h23456789;
		send_en = 1;
		#20;
		send_en = 0;
		#20;
		@(posedge Tx_Done);
		#1;
		#2000;
		$stop;		
	end

endmodule
