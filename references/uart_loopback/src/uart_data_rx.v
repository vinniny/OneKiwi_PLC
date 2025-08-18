/*作者：小梅哥 武汉芯路恒科技有限公司*/
/*=====================================
本模块实现1~16字节（8~256）位数据的接收。实现将多个字节的数据接收并拼接为指定的位宽数据后输出，
接收输出的数据位宽，可以在例化模块时使用DATA_WIDTH来修改，主要应用场景为数据位宽超过8位数据，希
望通过串口发送出去的场景，例如12位ADC采样结果通过串口发送。
要求DATA_WIDTH的值为8的整数倍，且最大不超过256。对于实际数据位宽达不到8的整数倍的情况，需要将数据
高位补0，以得到8整数倍位宽的数据之后再发送，例如对于12位ADC采样结果wire [11:0]ADC_DATA;使用时，可
以用如下写法
assign data = {4'd0,ADC_DATA},
其中data位宽为16，连接到uart_data_tx的data端口上作为待发送数据

**/

/*
例化模板
---------------------------------------
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
---------------------------------------
例化时
1、通过修改DATA_WIDTH的值来指定每次发送的数据位宽
2、通过修改MSB_FIRST的值来确定先发高字节还是先发低字节。为1则先发高字节，为0则先发低字节
3、send_en为脉冲触发信号，发送时提供一个时钟周期的高脉冲即可触发一次传输
4、Baud_Set为波特率设置值，0（9600）、1（19200）、2（38400）、3（57600）、4（115200）
5、每次传输完成（指定位宽的数据传输完成），TX-Done产生一个时钟周期的高脉冲
======================================*/

module uart_data_rx(
	Clk,
	Rst_n,
  	uart_rx,
	
	data,
	Rx_Done,
	timeout_flag,
	
	Baud_Set
);
	
	parameter DATA_WIDTH = 16;
	parameter MSB_FIRST = 1;

	input Clk;
	input Rst_n;
	input uart_rx;
	
	output reg [DATA_WIDTH - 1 : 0]data;
	input [2:0]Baud_Set;

	output reg Rx_Done;
	output reg timeout_flag;
	
	reg [DATA_WIDTH - 1 : 0]data_r;

	wire [7:0] data_byte;
	wire byte_rx_done;
	wire [19:0] TIMEOUT;
	
	uart_byte_rx uart_byte_rx(
		.Clk(Clk),
		.Reset_n(Rst_n),
		.uart_rx(uart_rx),
		.rx_data(data_byte),
		.rx_done(byte_rx_done)
	);

	//根据波特率自动设置超时时间
    assign TIMEOUT = (Baud_Set == 3'd0) ? 20'd182291:
                     (Baud_Set == 3'd1) ? 20'd91145:
                     (Baud_Set == 3'd2) ? 20'd45572:
                     (Baud_Set == 3'd3) ? 20'd30381:
                                          20'd15190;
	
	reg [8:0]cnt;
	reg [1:0]state;
	
	localparam S0 = 0;	//等待单字节接收完成信号
	localparam S1 = 1;	//判断接收是否超时
	localparam S2 = 2;	//检查所有数据是否接收完成

	reg [31:0]timeout_cnt;
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)
		timeout_flag <= 1'd0;
    else if(timeout_cnt >= TIMEOUT)
		timeout_flag <= 1'd1;
	else if(state == S0)
	    timeout_flag <= 1'd0;
		
	reg to_state;
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)
	   	to_state <= 0;
	else if(!uart_rx)
	   to_state <= 1;
    else if(byte_rx_done)
        to_state <= 0; 
	
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)
		timeout_cnt <= 32'd0;
	else if(to_state)begin
        if(byte_rx_done)
            timeout_cnt <= 32'd0;
        else if(timeout_cnt >= TIMEOUT)
            timeout_cnt <= TIMEOUT;
        else
            timeout_cnt <= timeout_cnt + 1'd1;
    end
    
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)begin
		data_r <= 0;
		state <= S0;
		cnt <= 0;
		data <= 0;
	end
	else begin
		case(state)
			S0: 
				begin
					Rx_Done <= 0;
					data_r <= 0;
					if(DATA_WIDTH == 8)begin
						data <= data_byte;
						Rx_Done <= byte_rx_done;
					end
					else if(byte_rx_done)begin
						state <= S1;
						cnt <= cnt + 9'd8;
						if(MSB_FIRST == 1)
							data_r <= {data_r[DATA_WIDTH - 1 - 8 : 0], data_byte};
						else
							data_r <= {data_byte, data_r[DATA_WIDTH - 1 : 8]};
					end
				end
			
			S1:
				if(timeout_flag)begin
					state <= S0;
					Rx_Done <= 1;	
				end
				else if(byte_rx_done)begin
					state <= S2;
					cnt <= cnt + 9'd8;
					if(MSB_FIRST == 1)
						data_r <= {data_r[DATA_WIDTH - 1 - 8 : 0], data_byte};
					else
						data_r <= {data_byte, data_r[DATA_WIDTH - 1 : 8]};
				end
				
			S2:
				if(cnt >= DATA_WIDTH)begin
					state <= S0;
					cnt <= 0;
					data <= data_r;
					Rx_Done <= 1;
				end
				else begin
					state <= S1;
					Rx_Done <= 0;
				end
			default:state <= S0;
		endcase	
	end

endmodule
