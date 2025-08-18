`timescale 1ns / 1ps

module uart_byte_rx(
    Clk,
    Reset_n,
    uart_rx,
    rx_data,
    rx_done
);
    input Clk;
    input Reset_n;
    input uart_rx;
    output reg [7:0]rx_data;
    output reg rx_done;
    
    parameter BPS_SET = 433; //50000000/2000000 - 1
    
    reg [7:0]r_rx_data; 
    
    reg[1:0]rx_state;
    
    reg [2:0]uart_rx_reg;
    always@(posedge Clk)
        uart_rx_reg[2:0] <= {uart_rx_reg[1:0],uart_rx};
//        uart_rx_reg[2] <= uart_rx_reg[1]
//        uart_rx_reg[1] <= uart_rx_reg[0]
//        uart_rx_reg[0] <= uart_rx;
        
        
//    always@(posedge Clk)
//        uart_rx_reg <= {uart_rx_reg << 1,uart_rx};

    wire uart_rx_nedge;
    assign uart_rx_nedge = (uart_rx_reg[2:1] == 2'b10);
    
    localparam IDLE = 0;
    localparam RECEIVE = 1;
    
    reg [15:0]cnt1;
    
    reg [12:0]div_cnt;
    always@(posedge Clk or negedge Reset_n)
    if(!Reset_n)
        div_cnt <= 0;
    else if(rx_state == RECEIVE)begin
        if(div_cnt == BPS_SET-1)
            div_cnt <= 0;
        else
            div_cnt <= div_cnt + 1'd1;
    end
    else
        div_cnt <= 0;  
        
        
   wire bps_clk;
   
   assign bps_clk = (div_cnt == BPS_SET/2-1);
    
   reg [3:0]dcnt;
   always@(posedge Clk or negedge Reset_n)
   if(!Reset_n)   
         dcnt <= 0;
   else if(rx_state == RECEIVE)begin
        if(bps_clk)
            dcnt <= dcnt + 1'd1;
        else
            dcnt <= dcnt;
   end
   else
       dcnt <= 0; 
    
    //assign receive_done = cnt1 == 52083-1;
//    assign rx_done = (dcnt == 9) && (bps_clk);//5208.333*9.5

    always@(posedge Clk or negedge Reset_n)
    if(!Reset_n)
        rx_done <= 0;
    else if((dcnt == 9) && (bps_clk))
        rx_done <= 1;
    else
        rx_done <= 0;
    
    always@(posedge Clk or negedge Reset_n)
    if(!Reset_n)
        rx_data <= 0;
    else if((dcnt == 9) && (bps_clk))
        rx_data <= r_rx_data;
    //串口接收有个状态，空闲态和接收态

    always@(posedge Clk or negedge Reset_n)
    if(!Reset_n)
        rx_state <= IDLE;
    else begin
        case(rx_state)
           IDLE:
             if(uart_rx_nedge)
                rx_state <= RECEIVE;
             else
                rx_state <= IDLE;
                
          RECEIVE:
            if(rx_done)  
                rx_state <= IDLE;
            else
                rx_state <= RECEIVE;
          
          default: rx_state <= IDLE;
       endcase
    end
    
    
//    always@(posedge Clk or negedge Reset_n)
//    if(!Reset_n)
//        cnt1 <= 0;
//    else if(rx_state == RECEIVE)begin
//        if(receive_done)
//            cnt1 <= 0;
//        else
//            cnt1 <= cnt1 + 1'd1;
//    end
//    else
//        cnt1 <= 0;
     
    always@(posedge Clk or negedge Reset_n)
    if(!Reset_n)   begin
        r_rx_data <= 0;
    end
    else begin
        case(dcnt)
            1:r_rx_data[0] <= uart_rx_reg[1];
            2:r_rx_data[1] <= uart_rx_reg[1];
            3:r_rx_data[2] <= uart_rx_reg[1];
            4:r_rx_data[3] <= uart_rx_reg[1];
            5:r_rx_data[4] <= uart_rx_reg[1];
            6:r_rx_data[5] <= uart_rx_reg[1];
            7:r_rx_data[6] <= uart_rx_reg[1];
            8:r_rx_data[7] <= uart_rx_reg[1];
        endcase
    end

endmodule
