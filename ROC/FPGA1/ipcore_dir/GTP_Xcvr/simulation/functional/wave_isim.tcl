###############################################################################
## isim_wave.tcl
###############################################################################
onerror {resume}

wcfg new
divider add "FRAME CHECK MODULE tile0_frame_check0"
wave add /DEMO_TB/GTP_Xcvr_top_i/TILE0_frame_check0/begin_r
wave add /DEMO_TB/GTP_Xcvr_top_i/TILE0_frame_check0/track_data_r
wave add /DEMO_TB/GTP_Xcvr_top_i/TILE0_frame_check0/data_error_detected_r
wave add /DEMO_TB/GTP_Xcvr_top_i/TILE0_frame_check0/start_of_packet_detected_r
wave add /DEMO_TB/GTP_Xcvr_top_i/TILE0_frame_check0/RX_DATA
wave add /DEMO_TB/GTP_Xcvr_top_i/TILE0_frame_check0/ERROR_COUNT
divider add "FRAME CHECK MODULE tile0_frame_check1"
wave add /DEMO_TB/GTP_Xcvr_top_i/TILE0_frame_check1/begin_r
wave add /DEMO_TB/GTP_Xcvr_top_i/TILE0_frame_check1/track_data_r
wave add /DEMO_TB/GTP_Xcvr_top_i/TILE0_frame_check1/data_error_detected_r
wave add /DEMO_TB/GTP_Xcvr_top_i/TILE0_frame_check1/start_of_packet_detected_r
wave add /DEMO_TB/GTP_Xcvr_top_i/TILE0_frame_check1/RX_DATA
wave add /DEMO_TB/GTP_Xcvr_top_i/TILE0_frame_check1/ERROR_COUNT
divider add "PLL Ports"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/CLK00_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/CLK01_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/GTPRESET0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/GTPRESET1_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/PLLLKDET0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/PLLLKDET1_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RESETDONE0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RESETDONE1_OUT
divider add "Receive Ports - 8b10b Decoder"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXCHARISCOMMA0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXCHARISCOMMA1_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXCHARISK0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXCHARISK1_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXDISPERR0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXDISPERR1_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXNOTINTABLE0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXNOTINTABLE1_OUT
divider add "Receive Ports - Clock Correction"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXCLKCORCNT0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXCLKCORCNT1_OUT
divider add "Receive Ports - Comma Detection and Alignment"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXENMCOMMAALIGN0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXENMCOMMAALIGN1_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXENPCOMMAALIGN0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXENPCOMMAALIGN1_IN
divider add "Receive Ports - PRBS Detection"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/PRBSCNTRESET0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/PRBSCNTRESET1_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXENPRBSTST0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXENPRBSTST1_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXPRBSERR0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXPRBSERR1_OUT
divider add "Receive Ports - RX Data Path interface"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXDATA0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXDATA1_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXRESET0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXRESET1_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXUSRCLK0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXUSRCLK1_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXUSRCLK20_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXUSRCLK21_IN
divider add "Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXN0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXN1_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXP0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXP1_IN
divider add "Receive Ports - RX Elastic Buffer and Phase Alignment"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXBUFSTATUS0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXBUFSTATUS1_OUT
divider add "Receive Ports - RX Loss-of-sync State Machine"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXLOSSOFSYNC0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXLOSSOFSYNC1_OUT
divider add "Receive Ports - RX Polarity Control"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXPOLARITY0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/RXPOLARITY1_IN
divider add "TX/RX Datapath Ports"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/GTPCLKOUT0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/GTPCLKOUT1_OUT
divider add "Transmit Ports - 8b10b Encoder Control"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXCHARISK0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXCHARISK1_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXKERR0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXKERR1_OUT
divider add "Transmit Ports - TX Data Path interface"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXDATA0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXDATA1_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXUSRCLK0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXUSRCLK1_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXUSRCLK20_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXUSRCLK21_IN
divider add "Transmit Ports - TX Driver and OOB signalling"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXN0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXN1_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXP0_OUT
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXP1_OUT
divider add "Transmit Ports - TX PRBS Generator"
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXENPRBSTST0_IN
wave add /DEMO_TB/GTP_Xcvr_top_i/GTP_Xcvr_i/tile0_GTP_Xcvr_i/TXENPRBSTST1_IN


run 50 us
quit




