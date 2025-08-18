module gw_gao(
    uart_rx,
    uart_tx,
    Rx_Done,
    \rx_data[31] ,
    \rx_data[30] ,
    \rx_data[29] ,
    \rx_data[28] ,
    \rx_data[27] ,
    \rx_data[26] ,
    \rx_data[25] ,
    \rx_data[24] ,
    \rx_data[23] ,
    \rx_data[22] ,
    \rx_data[21] ,
    \rx_data[20] ,
    \rx_data[19] ,
    \rx_data[18] ,
    \rx_data[17] ,
    \rx_data[16] ,
    \rx_data[15] ,
    \rx_data[14] ,
    \rx_data[13] ,
    \rx_data[12] ,
    \rx_data[11] ,
    \rx_data[10] ,
    \rx_data[9] ,
    \rx_data[8] ,
    \rx_data[7] ,
    \rx_data[6] ,
    \rx_data[5] ,
    \rx_data[4] ,
    \rx_data[3] ,
    \rx_data[2] ,
    \rx_data[1] ,
    \rx_data[0] ,
    Clk,
    tms_pad_i,
    tck_pad_i,
    tdi_pad_i,
    tdo_pad_o
);

input uart_rx;
input uart_tx;
input Rx_Done;
input \rx_data[31] ;
input \rx_data[30] ;
input \rx_data[29] ;
input \rx_data[28] ;
input \rx_data[27] ;
input \rx_data[26] ;
input \rx_data[25] ;
input \rx_data[24] ;
input \rx_data[23] ;
input \rx_data[22] ;
input \rx_data[21] ;
input \rx_data[20] ;
input \rx_data[19] ;
input \rx_data[18] ;
input \rx_data[17] ;
input \rx_data[16] ;
input \rx_data[15] ;
input \rx_data[14] ;
input \rx_data[13] ;
input \rx_data[12] ;
input \rx_data[11] ;
input \rx_data[10] ;
input \rx_data[9] ;
input \rx_data[8] ;
input \rx_data[7] ;
input \rx_data[6] ;
input \rx_data[5] ;
input \rx_data[4] ;
input \rx_data[3] ;
input \rx_data[2] ;
input \rx_data[1] ;
input \rx_data[0] ;
input Clk;
input tms_pad_i;
input tck_pad_i;
input tdi_pad_i;
output tdo_pad_o;

wire uart_rx;
wire uart_tx;
wire Rx_Done;
wire \rx_data[31] ;
wire \rx_data[30] ;
wire \rx_data[29] ;
wire \rx_data[28] ;
wire \rx_data[27] ;
wire \rx_data[26] ;
wire \rx_data[25] ;
wire \rx_data[24] ;
wire \rx_data[23] ;
wire \rx_data[22] ;
wire \rx_data[21] ;
wire \rx_data[20] ;
wire \rx_data[19] ;
wire \rx_data[18] ;
wire \rx_data[17] ;
wire \rx_data[16] ;
wire \rx_data[15] ;
wire \rx_data[14] ;
wire \rx_data[13] ;
wire \rx_data[12] ;
wire \rx_data[11] ;
wire \rx_data[10] ;
wire \rx_data[9] ;
wire \rx_data[8] ;
wire \rx_data[7] ;
wire \rx_data[6] ;
wire \rx_data[5] ;
wire \rx_data[4] ;
wire \rx_data[3] ;
wire \rx_data[2] ;
wire \rx_data[1] ;
wire \rx_data[0] ;
wire Clk;
wire tms_pad_i;
wire tck_pad_i;
wire tdi_pad_i;
wire tdo_pad_o;
wire tms_i_c;
wire tck_i_c;
wire tdi_i_c;
wire tdo_o_c;
wire [9:0] control0;
wire gao_jtag_tck;
wire gao_jtag_reset;
wire run_test_idle_er1;
wire run_test_idle_er2;
wire shift_dr_capture_dr;
wire update_dr;
wire pause_dr;
wire enable_er1;
wire enable_er2;
wire gao_jtag_tdi;
wire tdo_er1;

IBUF tms_ibuf (
    .I(tms_pad_i),
    .O(tms_i_c)
);

IBUF tck_ibuf (
    .I(tck_pad_i),
    .O(tck_i_c)
);

IBUF tdi_ibuf (
    .I(tdi_pad_i),
    .O(tdi_i_c)
);

OBUF tdo_obuf (
    .I(tdo_o_c),
    .O(tdo_pad_o)
);

GW_JTAG  u_gw_jtag(
    .tms_pad_i(tms_i_c),
    .tck_pad_i(tck_i_c),
    .tdi_pad_i(tdi_i_c),
    .tdo_pad_o(tdo_o_c),
    .tck_o(gao_jtag_tck),
    .test_logic_reset_o(gao_jtag_reset),
    .run_test_idle_er1_o(run_test_idle_er1),
    .run_test_idle_er2_o(run_test_idle_er2),
    .shift_dr_capture_dr_o(shift_dr_capture_dr),
    .update_dr_o(update_dr),
    .pause_dr_o(pause_dr),
    .enable_er1_o(enable_er1),
    .enable_er2_o(enable_er2),
    .tdi_o(gao_jtag_tdi),
    .tdo_er1_i(tdo_er1),
    .tdo_er2_i(1'b0)
);

gw_con_top  u_icon_top(
    .tck_i(gao_jtag_tck),
    .tdi_i(gao_jtag_tdi),
    .tdo_o(tdo_er1),
    .rst_i(gao_jtag_reset),
    .control0(control0[9:0]),
    .enable_i(enable_er1),
    .shift_dr_capture_dr_i(shift_dr_capture_dr),
    .update_dr_i(update_dr)
);

ao_top_0  u_la0_top(
    .control(control0[9:0]),
    .trig0_i(Rx_Done),
    .data_i({uart_rx,uart_tx,Rx_Done,\rx_data[31] ,\rx_data[30] ,\rx_data[29] ,\rx_data[28] ,\rx_data[27] ,\rx_data[26] ,\rx_data[25] ,\rx_data[24] ,\rx_data[23] ,\rx_data[22] ,\rx_data[21] ,\rx_data[20] ,\rx_data[19] ,\rx_data[18] ,\rx_data[17] ,\rx_data[16] ,\rx_data[15] ,\rx_data[14] ,\rx_data[13] ,\rx_data[12] ,\rx_data[11] ,\rx_data[10] ,\rx_data[9] ,\rx_data[8] ,\rx_data[7] ,\rx_data[6] ,\rx_data[5] ,\rx_data[4] ,\rx_data[3] ,\rx_data[2] ,\rx_data[1] ,\rx_data[0] }),
    .clk_i(Clk)
);

endmodule
