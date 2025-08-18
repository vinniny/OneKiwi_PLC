module uart_loopback(
    Clk,
    Rst_n,
    uart_rx,
    
    led,
    uart_tx
 );
 
    parameter DATA_WIDTH = 32;
	parameter MSB_FIRST = 0;
	
    input Clk;
    input Rst_n;
    input uart_rx;
    
    output [2:0]led;
    output uart_tx;
    
    wire [DATA_WIDTH-1:0]rx_data;
    wire Rx_Done;
    wire [7:0]data_byte;


    uart_data_rx 
    #(
		.DATA_WIDTH(DATA_WIDTH),
		.MSB_FIRST(MSB_FIRST)		
	)
	uart_data_rx(
        .Clk(Clk),
        .Rst_n(Rst_n),
        .uart_rx(uart_rx),
        
        .data(rx_data),
        .Rx_Done(Rx_Done),
        .timeout_flag(led[0]),
        
        .Baud_Set(3'd4)
     );

    uart_data_tx 
    #(
		.DATA_WIDTH(DATA_WIDTH),
		.MSB_FIRST(MSB_FIRST)
	)uart_data_tx(
        .Clk(Clk),
        .Rst_n(Rst_n),
      
        .data(rx_data),
        .send_en(Rx_Done),   
        .Baud_Set(3'd4),  
        
        .uart_tx(uart_tx),  
        .Tx_Done(led[1]),   
        .uart_state(led[2])
    );
    
endmodule
