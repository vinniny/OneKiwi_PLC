/*作者：小梅哥 武汉芯路恒科技有限公司*/
/*=====================================
本模块实现1~16字节（8~256）位数据的发送。需要发送的数据位宽，可以在例化模块时使用DATA_WIDTH来修改
主要应用场景为数据位宽超过8位数据，希望通过串口发送出去的场景，例如12位ADC采样结果通过串口发送。

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


module uart_data_tx(
	Clk,
	Rst_n,
  
	data,
	send_en,   
	Baud_Set,  
	
	uart_tx,  
	Tx_Done,   
	uart_state
);
	
	parameter DATA_WIDTH = 8;
	parameter MSB_FIRST = 1;

	input Clk;
	input Rst_n;
	
	input [DATA_WIDTH - 1 : 0]data;
	input send_en;
	input [2:0]Baud_Set;
	output uart_tx;
	output reg Tx_Done;
	output uart_state;
	
	reg [DATA_WIDTH - 1 : 0]data_r;

	reg [7:0] data_byte;
	reg byte_send_en;
	wire byte_tx_done;
	
	uart_byte_tx uart_byte_tx(
		.Clk(Clk),
		.Rst_n(Rst_n),
		.data_byte(data_byte),
		.send_en(byte_send_en),   
		.Baud_Set(Baud_Set),  
		.uart_tx(uart_tx),  
		.Tx_Done(byte_tx_done),   
		.uart_state(uart_state) 
	);
	
	reg [8:0]cnt;
	reg [1:0]state;
	
	localparam S0 = 0;	//等待发送请求
	localparam S1 = 1;	//发起单字节数据发送
	localparam S2 = 2;	//等待单字节数据发送完成
	localparam S3 = 3;	//检查所有数据是否发送完成
	
	always@(posedge Clk or negedge Rst_n)
	if(!Rst_n)begin
		data_byte <= 0;
		byte_send_en <= 0;
		state <= S0;
		cnt <= 0;
	end
	else begin
		case(state)
			S0: 
				begin
					data_byte <= 0;
					cnt <= 0;
					Tx_Done <= 0;
					if(send_en)begin
						state <= S1;
						data_r <= data;
					end
					else begin
						state <= S0;
						data_r <= data_r;
					end
				end
			
			S1:
				begin
					byte_send_en <= 1;
					if(MSB_FIRST == 1)begin
						data_byte <= data_r[DATA_WIDTH-1:DATA_WIDTH - 8];
						data_r <= data_r << 8;
					end
					else begin
						data_byte <= data_r[7:0];
						data_r <= data_r >> 8;					
					end
					state <= S2;
				end
				
			S2:
				begin
					byte_send_en <= 0;
					if(byte_tx_done)begin
						state <= S3;
						cnt <= cnt + 9'd8;
					end
					else
						state <= S2;
				end
			
			S3:
				if(cnt >= DATA_WIDTH)begin
					state <= S0;
					cnt <= 0;
					Tx_Done <= 1;
				end
				else begin
					state <= S1;
					Tx_Done <= 0;
				end
			default:state <= S0;
		endcase	
	end

endmodule
