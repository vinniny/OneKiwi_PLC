/**
 * @file mb.v
 * @breif verilog implementation of Modbus slaver protocal.
 * @details Verilog implementation of FreeModbus.
 * @details FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * @auther denghb
 * @date 20190113
 * @version 1.0
 * @license GNU General Public License (GPL) 3.0 
 *
 *________________________________________________________
 * History:
 *  <Date>      |   <version>   |     <Author>      |     <Description>
 *  20190113    |   1.0	        |     denghb        |     first created
 *________________________________________________________
**/


//`define MB_RTU 0
`define MB_ASCII 1
//`define MB_TCP 2



module mb(

    // inputs
    inMBPoll,
    
    
    // inouts
	
	
    // outputs
	outEventState,
);  


parameter BaudRate = 115200;
parameter SlaveAddress = 8'h0A;
parameter EV_READY = 0;
parameter EV_FRAME_RECEIVED = 1;
parameter EV_EXECUTE = 2;
parameter EV_FRAME_SENT =3;


`ifdef MB_RTU
    /**
     * @brief MB_RTU code here
    **/
    assign eMode = MB_RTU;
    
`elsif MB_ASCII
    /**
     * @brief MB_ASCII code here
    **/
    assign eMode = MB_ASCII;
    
    
    
`elseif MB_TCP
    /**
     * @brief MB_TCP code here
    **/
    assign eMode = MB_TCP;

`endif
 
 
    /**
     * @brief MB protocal Event State Machine
    **/
    always @ ( posedge clk or negedge rst_n )
    begin
        if ( !rst_n )
            stateEventCur <= EV_READY;
        else
            stateEventCur <= stateEventNext;
    end
    always @ *
    begin
        stateEventNext = stateEventCur;
        case (stateEventCur)
            EV_READY:
            begin
                
            end
            EV_FRAME_RECEIVED:
            begin
                
            end
            EV_EXECUTE:
            begin
                
            end
            EV_FRAME_SENT:
            begin
                
            end
            default:
            begin
                
            end
        endcase
    end
    always @ ( posedge clk or negedge rst_n )
    begin
        if ( !rst_n )
            eEvent <= EV_READY;
        else if ( UARTRxISR && ( eRcvState == STATE_RX_WAIT_EOF ) && ucByte == ucMBLFCharacter )
            eEvent <= EV_FRAME_RECEIVED;
        else if ( inMBPoll && UARTRxISR && ( usRcvBufferPos >= MB_SER_PDU_SIZE_MIN ) && ( prvucMBLRC( ( UCHAR * ) ucASCIIBuf, usRcvBufferPos ) == 0 ) && ( ( ucRcvAddress == ucMBAddress ) || ( ucRcvAddress == MB_ADDRESS_BROADCAST ) ) )
            eEvent <= EV_EXECUTE;
        else if ( inMBPoll && UARTTxISR && ( eSndState == STATE_TX_NOTIFY ) )
            eEvent <= EV_FRAME_SENT;
    end

    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            eMBErrorCode <= MB_ENOERR;
    end

    
endmodule
