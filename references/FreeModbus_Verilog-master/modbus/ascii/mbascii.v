/**
 * @file mbascii.v
 * @breif ascii transceiver state machine.
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


module mbascii(
    // inputs
    clk,
    rst_n,
    inMode,
    inByte
    
    
    // inouts
    
    
    // outputs
    outByte
    outRcvState,
    outSndState,
);


    // State parameter for @var regRcvState
    parameter STATE_RX_IDLE = 0;
    parameter STATE_RX_RCV = 1;
    parameter STATE_RX_WAIT_EOF = 2;
    
    
    // State parameter for @var regSndState 
    parameter STATE_TX_IDLE = 0;
    parameter STATE_TX_START = 1;
    parameter STATE_TX_DATA = 2;
    parameter STATE_TX_END = 3
    parameter STATE_TX_NOTIFY = 4;
    
    
    // State parameter for @var regBytePos 
    parameter BYTE_HIGH_NIBBLE = 0;
    parameter BYTE_LOW_NIBBLE = 1;
    
    
    // inputs
    
    
    // inouts
    
    
    // outputs
    
    
    // wires
    
    
    // regs
    reg     [1:0]   regRcvState;
    reg     [2:0]   regSndState;
    reg             regBytePos;
    
    
    /** @defgroup ReceiveStateMachine
     *  @{
     **/
    always @ (posedge clk or negedge rst_n)
    begin
        if ( !rst_n )
            regRcvState <= STATE_RX_IDLE;
        else if ( UARTRxISR && ( inByte == ':' ) )
            regRcvState <= STATE_RX_RCV;
        else if ( UARTRxISR && ( inByte == MB_ASCII_DEFAULT_CR ) )
            regRcvState <= STATE_RX_WAIT_EOF;
        else if (UARTRxISR && xMBPortEventPost)
            regRcvState <= STATE_RX_IDLE;
    end

    
    /** @defgroup SendStateMachine
     *  @{
     **/
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            regRcvState <= STATE_TX_IDLE;
        else if (xMBPortEventPost)
            regRcvState <= STATE_TX_START;
        else if (xMBPortEventPost)
            regRcvState <= STATE_TX_DATA;
        else if (xMBPortEventPost)
            regRcvState <= STATE_TX_END;
        else if ()
            regRcvState <= STATE_TX_NOTIFY;
        else if ()
            regRcvState <= STATE_TX_IDLE;
    end
    
    
    /** @defgroup BytePositionMachine
     *  @{
     **/
    always @ (posedge clk or negedge rst_n)
    begin
        if (!rst_n)
            regBytePos <= BYTE_HIGH_NIBBLE;
        else if ()
            regBytePos <= BYTE_HIGH_NIBBLE;
        else if ()
            regBytePos <= BYTE_LOW_NIBBLE;
    end
    
    
    // assign outpus
    assign outRcvState = regRcvState;
    assign outSndState = regSndState;
    
    
endmodule

