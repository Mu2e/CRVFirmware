
###############################################################################
## wave_ncsim.sv
###############################################################################

  window new WaveWindow  -name  "Waves for Spartan-6 GTP Wizard Example Design"
  waveform  using  "Waves for Spartan-6 GTP Wizard Example Design"
  
  waveform  add  -label FRAME_CHECK_MODULE -comment tile0_frame_check0
  waveform  add  -signals  :gtp_xcvr_top_i+delimiter+tile0_frame_check0:begin_r
  waveform  add  -signals  :gtp_xcvr_top_i+delimiter+tile0_frame_check0:track_data_r
  waveform  add  -siganls  :gtp_xcvr_top_i+delimiter+tile0_frame_check0:data_error_detected_r
  wavefrom  add  -siganls  :gtp_xcvr_top_i+delimiter+tile0_frame_check0:start_of_packet_detected_r
  waveform  add  -signals  :gtp_xcvr_top_i+delimiter+tile0_frame_check0:RX_DATA
  waveform  add  -signals  :gtp_xcvr_top_i+delimiter+tile0_frame_check0:ERROR_COUNT
  waveform  add  -label FRAME_CHECK_MODULE -comment tile0_frame_check1
  waveform  add  -signals  :gtp_xcvr_top_i+delimiter+tile0_frame_check1:begin_r
  waveform  add  -signals  :gtp_xcvr_top_i+delimiter+tile0_frame_check1:track_data_r
  waveform  add  -siganls  :gtp_xcvr_top_i+delimiter+tile0_frame_check1:data_error_detected_r
  wavefrom  add  -siganls  :gtp_xcvr_top_i+delimiter+tile0_frame_check1:start_of_packet_detected_r
  waveform  add  -signals  :gtp_xcvr_top_i+delimiter+tile0_frame_check1:RX_DATA
  waveform  add  -signals  :gtp_xcvr_top_i+delimiter+tile0_frame_check1:ERROR_COUNT

  waveform  add  -label TILE0_GTP_Xcvr -comment TILE0_GTP_Xcvr
  waveform  add  -label PLL_Ports  -comment  PLL_Ports
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:CLK00_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:CLK01_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:GTPRESET0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:GTPRESET1_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:PLLLKDET0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:PLLLKDET1_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RESETDONE0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RESETDONE1_OUT
  waveform  add  -label Receive_Ports_-_8b10b_Decoder  -comment  Receive_Ports_-_8b10b_Decoder
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXCHARISCOMMA0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXCHARISCOMMA1_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXCHARISK0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXCHARISK1_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXDISPERR0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXDISPERR1_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXNOTINTABLE0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXNOTINTABLE1_OUT
  waveform  add  -label Receive_Ports_-_Clock_Correction  -comment  Receive_Ports_-_Clock_Correction
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXCLKCORCNT0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXCLKCORCNT1_OUT
  waveform  add  -label Receive_Ports_-_Comma_Detection_and_Alignment  -comment  Receive_Ports_-_Comma_Detection_and_Alignment
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXENMCOMMAALIGN0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXENMCOMMAALIGN1_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXENPCOMMAALIGN0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXENPCOMMAALIGN1_IN
  waveform  add  -label Receive_Ports_-_PRBS_Detection  -comment  Receive_Ports_-_PRBS_Detection
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:PRBSCNTRESET0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:PRBSCNTRESET1_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXENPRBSTST0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXENPRBSTST1_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXPRBSERR0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXPRBSERR1_OUT
  waveform  add  -label Receive_Ports_-_RX_Data_Path_interface  -comment  Receive_Ports_-_RX_Data_Path_interface
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXDATA0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXDATA1_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXRESET0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXRESET1_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXUSRCLK0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXUSRCLK1_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXUSRCLK20_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXUSRCLK21_IN
  waveform  add  -label Receive_Ports_-_RX_Driver,OOB_signalling,Coupling_and_Eq.,CDR  -comment  Receive_Ports_-_RX_Driver,OOB_signalling,Coupling_and_Eq.,CDR
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXN0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXN1_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXP0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXP1_IN
  waveform  add  -label Receive_Ports_-_RX_Elastic_Buffer_and_Phase_Alignment  -comment  Receive_Ports_-_RX_Elastic_Buffer_and_Phase_Alignment
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXBUFSTATUS0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXBUFSTATUS1_OUT
  waveform  add  -label Receive_Ports_-_RX_Loss-of-sync_State_Machine  -comment  Receive_Ports_-_RX_Loss-of-sync_State_Machine
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXLOSSOFSYNC0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXLOSSOFSYNC1_OUT
  waveform  add  -label Receive_Ports_-_RX_Polarity_Control  -comment  Receive_Ports_-_RX_Polarity_Control
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXPOLARITY0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:RXPOLARITY1_IN
  waveform  add  -label TX/RX_Datapath_Ports  -comment  TX/RX_Datapath_Ports
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:GTPCLKOUT0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:GTPCLKOUT1_OUT
  waveform  add  -label Transmit_Ports_-_8b10b_Encoder_Control  -comment  Transmit_Ports_-_8b10b_Encoder_Control
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXCHARISK0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXCHARISK1_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXKERR0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXKERR1_OUT
  waveform  add  -label Transmit_Ports_-_TX_Data_Path_interface  -comment  Transmit_Ports_-_TX_Data_Path_interface
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXDATA0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXDATA1_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXUSRCLK0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXUSRCLK1_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXUSRCLK20_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXUSRCLK21_IN
  waveform  add  -label Transmit_Ports_-_TX_Driver_and_OOB_signalling  -comment  Transmit_Ports_-_TX_Driver_and_OOB_signalling
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXN0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXN1_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXP0_OUT
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXP1_OUT
  waveform  add  -label Transmit_Ports_-_TX_PRBS_Generator  -comment  Transmit_Ports_-_TX_PRBS_Generator
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXENPRBSTST0_IN
  waveform  add  -signals  :GTP_Xcvr_top_i:GTP_Xcvr_i:tile0_GTP_Xcvr_i:TXENPRBSTST1_IN

  console submit -using simulator -wait no "run 50 us"

