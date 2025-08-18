module uart_data_rx_tb();

    parameter DATA_WIDTH = 32;
    parameter MSB_FIRST = 0;

    reg Clk;
    reg Rst_n;
    
    reg [DATA_WIDTH - 1 : 0]data;
    wire [DATA_WIDTH - 1 : 0]rx_data;
    reg send_en;
    wire uart_tx;
    wire uart_rx;
    wire Tx_Done;
    wire uart_state;
    wire Rx_Done;
    wire timeout_flag;
    
    assign uart_rx = uart_tx;
    
    //为多字节串口接收模块产生输入数据
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
        .timeout_flag(timeout_flag),
        .Baud_Set(3'd4)
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
        data = 32'h12345678;
        send_en = 1;
        #20;
        send_en = 0;
        #20;
        @(posedge Tx_Done);
        #1;
        #1000000;
        data = 32'h87654321;
        send_en = 1;
        #20;
        send_en = 0;
        #20;
        @(posedge Tx_Done);
        #1; 
        #1000000;
        data = 32'h24680135;
        send_en = 1;
        #20;
        send_en = 0;
        #20;
        @(posedge Tx_Done);
        #1;
        #400000;
        $stop;      
    end

endmodule
