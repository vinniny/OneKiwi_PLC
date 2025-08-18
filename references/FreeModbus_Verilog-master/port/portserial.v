/**
 * @file portserial.v
 * @breif driver for serial port.
 * @auther denghb
 * @date 20190119
 * @version 1.0
 * @license GNU General Public License (GPL) 3.0 
 *
 *_______________________________________________________________________
 * History:
 *  <Date>      |   <version>   |     <Author>      |     <Description>
 *  20190119    |   1.0	        |     denghb        |     first created
 *_______________________________________________________________________
**/



module portserial(
    // inputs
    inMBPortSerialPutByte,
    inMBPortSerialGetByte,
    inBaudRate,
    inByte,
    
    // inouts
    
    
    // outpus
    outRxEnable,
    outTxEnable,
    outByte,
    outUARTTxReadyISR,
    outUARTRxISR,
);


    // parameters

    
    // 
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            xRxEnable <= FALSE;
        else if ()
            xRxEnable <= TRUE;
        else if ()
            xRxEnable <= FALSE;
    end
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            xTxEnable <= FALSE;
        else if ()
            xTxEnable <= TRUE;
        else if ()
            xTxEnable <= FALSE;
    end
    

    /**
     * Put uart module here
     **/


endmodule

