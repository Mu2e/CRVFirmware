LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

Library UNISIM;
use UNISIM.vcomponents.all;

use work.Project_defs.all;

entity PacketFormer is
    port (
        clk              : in  std_logic; 
        reset            : in  std_logic; -- active high
        pktFormerSend    : in  std_logic; -- trigers package
        pktFormerTimeout : in  std_logic; 
		  FormRst          : in  std_logic; 
        EventBuff_Out     : in  std_logic_vector(15 downto 0);
        EventBuff_RdEn   : out std_logic; 
        TStmpBuff_rd_en   : out std_logic;
        TStmpBuff_Out     : in  std_logic_vector(15 downto 0);
		  EventBuff_Empty   : in  std_logic;
        ActiveReg        : in  std_logic_vector(23 downto 0);
        DCSBuffRdCnt      : in  std_logic_vector(12 downto 0);
        DCSBuff_Out       : in  std_logic_vector(15 downto 0);
        DCSBuff_rd_en     : out std_logic;
        DRdone           : out std_logic; -- used to clear DR handler  
        TxCRCEn          : out std_logic;
        TxCRCRst         : out std_logic;
		  TxCRCDat         : out std_logic_vector(15 downto 0);
		  TxCRC            : in  std_logic_vector(15 downto 0);
        WdCountBuff_WrtEn : out std_logic;
        GTPTx            : out std_logic_vector(15 downto 0);
        TxCharIsK        : out std_logic_vector( 1 downto 0);
        GTPTxBuff_In      : out std_logic_vector(15 downto 0); -- why not just use GTPTx?
		  GTPTxBuff_wr_en   : out std_logic;
		  --Marker           : in  std_logic;
		  MarkerDelayed    : in  std_logic;
		  loopbackMarker   : in  std_logic;
        -- settings
        LoopbackMode     : in  std_logic_vector( 2 downto 0);
		  uBwrt            : in  std_logic;
        IDReg            : in  std_logic_vector( 3 downto 0)
    );
end entity PacketFormer;

architecture rtl of PacketFormer is
    Type Packet_Former_Seq is (Idle,WrtPktCnt,WrtPktCntTimeout,WrtHdrPkt,WrtHdrPktTimeout,WrtCtrlHdrPkt,
	                            WrtDatPkt,WrtDCSPkt,WrtGRPkt,WrtGRPkt2,
	                            Loopback,Loopback2);
    signal current_state, next_state : Packet_Former_Seq;
    --signal FormStatRegNext : std_logic_vector(2 downto 0);
    signal FormStatReg     : std_logic_vector(2 downto 0);
    signal TxPkCnt         : std_logic_vector(10 downto 0);
    signal EvTxWdCnt       : std_logic_vector(13 downto 0);
    signal EvTxWdCntTC     : std_logic;
    signal Pkt_Timer       : std_logic_vector( 3 downto 0);
    signal DReq_Count      : std_logic_vector(31 downto 0);
    signal EventBuffErr    : std_logic;
	 signal GTPTxStage      : std_logic_vector(15 downto 0);
	 signal TxSeqNo         : std_logic_vector( 2 downto 0);
	 signal GTPTxBuff_Pipe   : std_logic_vector(15 downto 0);
	 signal GTPTx_Pipe      : std_logic_vector(15 downto 0);
    signal TxCharIsK_Pipe  : std_logic_vector(1 downto 0);
	 signal MarkerSync      : std_logic;
begin

    -- Next state logic (combinational)
    next_state_process: process(current_state, pktFormerSend, pktFormerTimeout, EvTxWdCnt, FormStatReg, 
	                             FormRst, EventBuffErr, Pkt_Timer, MarkerSync)
										  -- MarkerSync
    begin
        Case current_state is
            when Idle =>                                          --FormStatRegNext <= "000"; 
            if    pktFormerSend = '1' and pktFormerTimeout = '0' then next_state <= WrtPktCnt;
            elsif pktFormerSend = '1' and pktFormerTimeout = '1' then next_state <= WrtPktCntTimeout; -- 
            elsif DCSBuffRdCnt > 1                                then next_state <= WrtDCSPkt;
				elsif MarkerSync = '1'                               then next_state <= Loopback;
            --elsif (LoopbackMode = "1" and Marker = '1') then Packet_Former <= Loopback;
            --elsif (LoopbackMode = 1)                             then next_state <= Loopback;
            --elsif (LoopbackMode = 2 and Marker = '1')            then next_state <= Loopback;
            --elsif (LoopbackMode = 3 and MarkerDelayed = '1')     then next_state <= Loopback;
            --elsif (LoopbackMode = 4 and loopbackMarker = '1')    then next_state <= Loopback; -- TODO, markerfromfiber
            else                                                      next_state <= Idle; --Debug(5 downto 3) <= "000";
            end if;
    -- Loopback
        when Loopback =>          next_state <= Loopback2;
        when Loopback2 =>
    --    --if (LoopbackMode = "0" or Marker = '0') then Packet_Former <= Idle;
        if MarkerSync = '0' then  next_state <= Idle;
        else                      next_state <= next_state;
        end if;
            
    -- Divide by eight to get the number of packets
        when WrtPktCnt        => next_state <= WrtHdrPkt;         --FormStatRegNext <= "001";  
        when WrtPktCntTimeout => next_state <= WrtHdrPktTimeout;  --FormStatRegNext <= "001";
    -- Send the packet header, packet type, packet count, time stamp and status
        when WrtHdrPkt =>                                        -- FormStatRegNext <= "010"; 
            if    Pkt_Timer = 0  then next_state <= WrtCtrlHdrPkt; --Debug(5 downto 3) <= "011";
             --if    Pkt_Timer = 0 and pktFormerTimeout = '0' then Packet_Former <= WrtCtrlHdrPkt; --Debug(5 downto 3) <= "011";
            --elsif Pkt_Timer = 0 and pktFormerTimeout = '1' then Packet_Former <= Idle;
            elsif FormRst = '1'  then next_state <= Idle; --Debug(5 downto 3) <= "000";
            else                      next_state <= WrtHdrPkt; --Debug(5 downto 3) <= "101";
            end if;
            
        when WrtHdrPktTimeout =>                                  --FormStatRegNext <= "010";
            if    Pkt_Timer = 0  then next_state <= Idle;
            else                      next_state <= WrtHdrPktTimeout;
            end if;
        
        when WrtCtrlHdrPkt =>                                     --FormStatRegNext <= "011";  
            if Pkt_Timer = 0 then 
                if EvTxWdCnt = 0 then next_state <= Idle;
                else                  next_state <= WrtDatPkt; --Debug(5 downto 3) <= "111";
                end if;
            elsif FormRst = '1'  then next_state <= Idle; --Debug(5 downto 3) <= "000";
            else                      next_state <= WrtCtrlHdrPkt; --Debug(5 downto 3) <= "011";
            end if;
        -- After Controller header is sent, the packets contain data for this FEB
        -- The FEB header data is embedded in the sream coming from the front FPGAs
        when WrtDatPkt =>                                         --FormStatRegNext <= "100";   
            if     EvTxWdCnt = 0 and Pkt_Timer = 0     then next_state <= Idle; --Debug(5 downto 3) <= "000";
            elsif EventBuffErr = '1' and Pkt_Timer = 0 then next_state <= Idle;
            elsif FormRst = '1'                        then next_state <= Idle; --Debug(5 downto 3) <= "000";
            else                                            next_state <= WrtDatPkt; --Debug(5 downto 3) <= "111";
            end if;
        when WrtDCSPkt =>                                         --FormStatRegNext <= "111";
            if FormStatReg = "111" and Pkt_Timer = 0   then next_state <= Idle;
            elsif FormRst = '1'                        then next_state <= Idle;
            else                                            next_state <= WrtDCSPkt;
            end if;
        --when WrtGRPkt => FormStatReg <= "111";
        --   if FormStatReg = "111" and Pkt_Timer = 0 then Packet_Former <= WrtGRPkt2;
        --	elsif FormRst = '1'                      then Packet_Former <= Idle;
        --	else                                          Packet_Former <= WrtGRPkt;
        --	end if;
        --when WrtGRPkt2 => FormStatReg <= "110";
        --   if FormStatReg = "110" and Pkt_Timer = 0 and DReqBuff_Emtpy = '0' -- "100" guard probably not needed
        --	    then Packet_Former <= Idle;
        --	elsif FormRst = '1' then Packet_Former <= Idle;
        --	else Packet_Former <= WrtGRPkt2;
        --	end if;
            
        when others =>
                                                            next_state <= Idle;
          end case;
    end process next_state_process;

    -- State register and output logic (sequential)
    state_and_output_process: process(clk, reset)
    begin
        if reset = '1' then
            current_state <= Idle;
            FormStatReg <= (others => '0');
            WdCountBuff_WrtEn <= '0';
            DRdone <= '0';
            TxPkCnt <= (others => '0');
            EvTxWdCnt <= (others => '0');
            EvTxWdCntTC <= '0';
            Pkt_Timer <= (others => '0');
            GTPTx <= X"BC3C";
				GTPTx_Pipe <= X"BC3C";
            GTPTxBuff_In <= X"BC3C";
				GTPTxBuff_Pipe <= X"BC3C";
            GTPTxStage <= X"BC3C";
            TxCRCDat <= (others => '0');
            TxCRCEn <= '0';
            TxCRCRst <= '1';
            TxSeqNo <= "000";
            TxCharIsK <= "11";
				TxCharIsK_Pipe <= "11";
            DReq_Count <= (others => '0');
            EventBuffErr <= '0';
            EventBuff_RdEn <= '0';
				DCSBuff_rd_en <= '0';
				TStmpBuff_rd_en <= '0';
				GTPTxBuff_wr_en <= '0';
				MarkerSync <= '0';

        elsif rising_edge(clk) then
            current_state <= next_state;
            --FormStatReg <= FormStatRegNext;
				
				if      LoopbackMode = 1
				    --or (LoopbackMode = 2 and Marker = '1') 
				    or (LoopbackMode = 3 and MarkerDelayed = '1')
					 or (LoopbackMode = 4 and loopbackMarker = '1')
					     then MarkerSync <= '1';
				else         MarkerSync <= '0';
				end if;
				
				Case current_state is
				    when Idle =>             FormStatReg <= "000"; 
					 when WrtPktCnt =>        FormStatReg <= "001";  
                when WrtPktCntTimeout => FormStatReg <= "001";
					 when WrtHdrPkt =>        FormStatReg <= "010"; 
					 when WrtHdrPktTimeout => FormStatReg <= "010";
					 when WrtDatPkt =>        FormStatReg <= "100";
					 when WrtDCSPkt =>        FormStatReg <= "111";
					 when others =>           FormStatReg <= "000";
				end case;

            if current_state = WrtPktCnt then WdCountBuff_WrtEn <= '1';
            else                              WdCountBuff_WrtEn <= '0';
            end if;
            
            -- this is used to advance the DR_Handler state machine
            if current_state = WrtHdrPkt or current_state = WrtHdrPktTimeout then DRdone <= '1'; --used 
            else                                                                  DRdone <= '0';
            end if;

            -- Sum word counts, divide by eight and add 1 for the controller header packet
            if current_state = WrtPktCnt and EventBuff_Out(2 downto 0)  = 0                        then TxPkCnt <= EventBuff_Out(13 downto 3) + 1;
            -- If not and even multiple of eight, account for final partially filled packet
            elsif current_state = WrtPktCnt and EventBuff_Out(2 downto 0) /= 0                     then TxPkCnt <= EventBuff_Out(13 downto 3) + 2;
            -- if timeout, only return data header
            -- elsif Packet_Former = WrtPktCntTimeout then TxPkCnt <= (others => '0');
            -- Decrement the packet count once for each packet ID write
            elsif ((current_state = WrtDatPkt or current_state = WrtCtrlHdrPkt) and Pkt_Timer = 9) then TxPkCnt <= TxPkCnt - 1;
            else                                                                                        TxPkCnt <= TxPkCnt;
            end if;
   
            -- Extract the word count from the event buffer FIFO 
            if current_state = WrtPktCnt                                         then EvTxWdCnt <= EventBuff_Out(13 downto 0);
            --elsif Packet_Former = WrtPktCntTimeout
            --   then EvTxWdCnt <= (others => '0'); -- no payload when timeout
            -- Decrement the word count for each word sent within a data packet
            elsif EvTxWdCnt /= 0 and current_state = WrtDatPkt and Pkt_Timer > 2 then EvTxWdCnt <= EvTxWdCnt - 1;
            else                                                                      EvTxWdCnt <= EvTxWdCnt;
            end if;
   
            -- Use this word count terminal count to distinguish the last valid word read
            -- from the event buffer FIFO
            if     EvTxWdCnt = 1 and current_state = WrtDatPkt and Pkt_Timer > 2 then EvTxWdCntTC <= '1';
            elsif EvTxWdCnt /= 1 and current_state = WrtDatPkt and Pkt_Timer > 2 then EvTxWdCntTC <= '0';
            else                                                                      EvTxWdCntTC <= EvTxWdCntTC;
            end if;
   
            -- Read of timestamps for use in forming the header packet
            if (current_state = WrtHdrPkt and Pkt_Timer <= 7 and Pkt_Timer >= 5) or
               (current_state = WrtHdrPktTimeout  and Pkt_Timer <= 7 and Pkt_Timer >= 5)
             --(current_state = WrtGRPkt  and Pkt_Timer <= 7 and Pkt_Timer >= 5)
            then    TStmpBuff_rd_en <= '1';
            else    TStmpBuff_rd_en <= '0';
            end if;
   
            -- Increment the data request counter when forming the header packet.
            if (current_state = WrtHdrPkt and Pkt_Timer = 9) or 
               (current_state = WrtHdrPktTimeout and Pkt_Timer = 9)
             --(Packet_Former = WrtGRPkt  and Pkt_Timer = 9)
                                                      then DReq_Count <= DReq_Count + 1;
            --elsif GTPRxRst = '1' or RxLOS(0)(1) = '1' then DReq_Count <= (others => '0'); -- let's reset with statemachine
            else                                           DReq_Count <= DReq_Count;
            end if;
   
            -- Counter for dividing data into packets
            if current_state = WrtPktCnt or current_state = WrtPktCntTimeout then Pkt_Timer <= X"A";
            elsif Pkt_Timer /= 0 and (current_state = WrtHdrPkt or current_state = WrtHdrPktTimeout
                or current_state = WrtCtrlHdrPkt or current_state = WrtDatPkt 
                or current_state = WrtDCSPkt 
                --or current_state = WrtGRPkt or current_state = WrtGRPkt2
                )
                                                                             then Pkt_Timer <= Pkt_Timer - 1;
            elsif Pkt_Timer = 0 and (current_state = WrtHdrPkt or current_state = WrtHdrPktTimeout -- WrtHdrPktTimeout not needed if we send only one package
                or current_state = WrtCtrlHdrPkt or current_state = WrtDatPkt 
                or current_state = WrtDCSPkt 
                --or current_state = WrtGRPkt or current_state = WrtGRPkt2
                )
                                                                            then Pkt_Timer <= X"A";
            elsif current_state = Idle                                      then Pkt_Timer <= X"0";
            else                                                                 Pkt_Timer <= Pkt_Timer;
            end if;

            if current_state = Loopback then
				     GTPTx_Pipe        <= X"1C90";
					  GTPTxBuff_Pipe     <= X"1C90";
				elsif Pkt_Timer = 0 and (
                  current_state = WrtHdrPkt 
              or  current_state = WrtHdrPktTimeout 
              or  current_state = WrtCtrlHdrPkt 
              or  current_state = WrtDatPkt 
              or (current_state = WrtDCSPkt and FormStatReg = "111")
              or (current_state = WrtGrPkt  and FormStatReg = "111")
              or (current_state = WrtGrPkt2)) -- and FormStatReg = "110"))
            then GTPTx_Pipe        <= TxCRC;      
                 GTPTxBuff_Pipe     <= TxCRC;
            else GTPTx_Pipe        <= GTPTxStage; 
                 GTPTxBuff_Pipe     <= GTPTxStage;
            end if;

            -------------------------------------------------------------------------------
            -- SC: removed the direct uC for the moment
            -------------------------------------------------------------------------------
            ---- Use this address to append K28.0 to Dx.y where x is 5 bits of data and
            ---- y is the packet sequence number to five bits of microcontroller data
            --if UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(0)
            --then GTPTx(0)     <= X"1C" & TxSeqNo(0) & uCD(4 downto 0);
            --     GTPTxBuff_In <= X"1C" & TxSeqNo(0) & uCD(4 downto 0);
            --     TxCRCDat(0)  <= X"0000";
            ---- Use this address to send unmodified microcontroller data
            --elsif UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(2)
            --then GTPTx(0) <= uCD; TxCRCDat(0) <= uCD;
            --GTPTxBuff_In <= uCD;
            ---- Use this address to send the check sum
            --elsif (UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(4))
            --then GTPTx(0) <= TxCRC(0); TxCRCDat(0) <= X"0000";
            --GTPTxBuff_In <= TxCRC(0);

            -- Data header packet ID field is 5 bits wide 
            -- The header packet ID is 5 
            --elsif Packet_Former = WrtHdrPkt 
            --then
            
            if current_state = WrtHdrPkt then
                Case Pkt_Timer is
                    When X"A" => GTPTxStage <= X"1C" & TxSeqNo & "00101";          TxCRCDat <= X"0000";
                    When X"8" => GTPTxStage <= X"8" & IDReg & X"50";               TxCRCDat <= X"8" & IDReg & X"50";
                    When X"7" => GTPTxStage <= "01000" & TxPkCnt;                  TxCRCDat <= "01000" & TxPkCnt;  
                    When X"6" => GTPTxStage <= TStmpBuff_Out;                      TxCRCDat <= TStmpBuff_Out;
                    When X"5" => GTPTxStage <= TStmpBuff_Out;                      TxCRCDat <= TStmpBuff_Out;
                    When X"4" => GTPTxStage <= TStmpBuff_Out;                      TxCRCDat <= TStmpBuff_Out;
                    When X"3" => GTPTxStage <= X"000" & "0" & EventBuffErr & "01"; TxCRCDat <= X"000" & "0" & EventBuffErr & "01";
                    When X"0" => GTPTxStage <= X"BC3C";                            TxCRCDat <= X"0000";
                    When others => GTPTxStage <= X"0000";                          TxCRCDat <= X"0000";
                end case;
            elsif current_state = WrtHdrPktTimeout then
                Case Pkt_Timer is
                    When X"A" => GTPTxStage <= X"1C" & TxSeqNo & "00101";          TxCRCDat <= X"0000";
                    When X"8" => GTPTxStage <= X"8" & IDReg & X"50";               TxCRCDat <= X"8" & IDReg & X"50";
                    When X"7" => GTPTxStage <= "01000" & "000" & X"00";            TxCRCDat <= "01000" & "000" & X"00"; 
                    When X"6" => GTPTxStage <= TStmpBuff_Out;                      TxCRCDat <= TStmpBuff_Out;
                    When X"5" => GTPTxStage <= TStmpBuff_Out;                      TxCRCDat <= TStmpBuff_Out;
                    When X"4" => GTPTxStage <= TStmpBuff_Out;                      TxCRCDat <= TStmpBuff_Out;
                    When X"3" => GTPTxStage <= X"000" & "1" & EventBuffErr & "00"; TxCRCDat <= X"000" & "1" & EventBuffErr & "00";
                    When X"0" => GTPTxStage <= X"BC3C";                            TxCRCDat <= X"0000";
                    When others => GTPTxStage <= X"0000";                          TxCRCDat <= X"0000";
                end case;
            elsif current_state = WrtCtrlHdrPkt then
                Case Pkt_Timer is
                    When X"A" => GTPTxStage <= X"1C" & TxSeqNo & "00110";          TxCRCDat <= X"0000";
                    When X"9" => GTPTxStage <= X"00" & X"6" & IDReg;               TxCRCDat <= X"00" & X"6" & IDReg; 
                    -- Add the words in the controller header packet to the total word count
                    When X"8" => GTPTxStage <= EventBuff_Out + 8;                  TxCRCDat <= EventBuff_Out + 8;
                    When X"7" => GTPTxStage <= X"00" & ActiveReg(23 downto 16);    TxCRCDat <= X"00" & ActiveReg(23 downto 16);
                    When X"6" => GTPTxStage <= ActiveReg(15 downto 0);             TxCRCDat <= ActiveReg(15 downto 0);
                    When X"5" => GTPTxStage <= DReq_Count(15 downto 0);            TxCRCDat <= DReq_Count(15 downto 0);
                    When X"4" => GTPTxStage <= EventBuff_Out;                      TxCRCDat <= EventBuff_Out; -- EventBuff_Out; -- word count 0
                    --When X"3" => GTPTxStage(0) <= uBcheckRef(31 downto 16); TxCRCDat(0) <= uBcheckRef(31 downto 16); -- EventBuff_Out; -- word count 1
                    --When X"2" => GTPTxStage(0) <= uBcheckRef(15 downto  0); TxCRCDat(0) <= uBcheckRef(15 downto  0); -- EventBuff_Out; -- word count 2
                    When X"3" => GTPTxStage <= EventBuff_Out;                      TxCRCDat <= EventBuff_Out; -- EventBuff_Out; -- word count 1
                    When X"2" => GTPTxStage <= EventBuff_Out;                      TxCRCDat <= EventBuff_Out; -- EventBuff_Out; -- word count 2
                    When X"0" => GTPTxStage <= X"BC3C";                            TxCRCDat <= X"0000";
                    When others => GTPTxStage <= X"0000";                          TxCRCDat <= X"0000";
                end case;
                elsif current_state = WrtDatPkt then 
                    if    Pkt_Timer = 10 then GTPTxStage <= X"1C" & TxSeqNo & "00110"; TxCRCDat <= X"0000";
                    elsif Pkt_Timer =  0 then GTPTxStage <= X"BC3C";               TxCRCDat <= X"0000";
                    elsif EvTxWdCnt > 0 or EvTxWdCntTC = '1' 
                              then GTPTxStage <= EventBuff_Out;                    TxCRCDat <= EventBuff_Out;
                    else GTPTxStage <= X"0000";                                    TxCRCDat <= X"0000";
                    end if;
                elsif current_state = WrtDCSPkt then
                    Case Pkt_Timer is
--			 --When X"A" => GTPTxStage(0) <= X"1C" & TxSeqNo(0) & "0" & DCS_Header(7 downto 4); TxCRCDat(0) <= X"0000"; 
                        When X"A" => GTPTxStage <= X"1C" & TxSeqNo & "0" & X"4";   TxCRCDat <= X"0000"; 
                        --When X"9" => GTPTxStage(0) <= DCS_EvCnt; TxCRCDat(0) <= DCS_EvCnt; 
                        When X"9" => GTPTxStage <= X"0010";                        TxCRCDat <= X"0010";
                        --When X"8" => GTPTxStage(0) <= DCS_Header; TxCRCDat(0) <= DCS_Header;
                        When X"8" => GTPTxStage <= X"8" & IDReg & X"40";           TxCRCDat <= X"8" & IDReg & X"40";
                        --When X"7" => GTPTxStage(0) <= DCS_Status; TxCRCDat(0) <= DCS_Status; 
                        When X"7" => GTPTxStage <= X"0010";                        TxCRCDat <= X"0010"; 			 
                        When X"6" => GTPTxStage <= DCSBuff_Out;                    TxCRCDat <= DCSBuff_Out;
                        When X"5" => GTPTxStage <= DCSBuff_Out;                    TxCRCDat <= DCSBuff_Out;
                        --When X"4" => GTPTxStage(0) <= X"0002"; TxCRCDat(0) <= X"0002";
                        --When X"3" => GTPTxStage(0) <= X"0003"; TxCRCDat(0) <= X"0003";
                        --When X"2" => GTPTxStage(0) <= X"0004"; TxCRCDat(0) <= X"0004";
                        When X"0" => GTPTxStage <= X"BC3C";                        TxCRCDat <= X"0000";
                        When others => GTPTxStage <= X"0000";                      TxCRCDat <= X"0000";
                    end case;		
                --elsif Packet_Former = WrtGRPkt then
                --		Case Pkt_Timer is
                --		 When X"A" => GTPTxStage(0) <= X"1C" & TxSeqNo(0) & "0" & X"5"; TxCRCDat(0) <= X"0000"; 
                --		 When X"9" => GTPTxStage(0) <= X"0020"; TxCRCDat(0) <= X"0020"; -- 16 words, 32 bytes, not needed?
                --		 When X"8" => GTPTxStage(0) <= X"8" & IDReg & X"50"; TxCRCDat(0) <= X"8" & IDReg & X"50"; -- 
                --		 When X"7" => GTPTxStage(0) <= X"4001"; TxCRCDat(0) <= X"4001"; -- one package 
                --		 When X"6" => GTPTxStage(0) <= TStmpBuff_Out; TxCRCDat(0) <= TStmpBuff_Out;
                --		 When X"5" => GTPTxStage(0) <= TStmpBuff_Out; TxCRCDat(0) <= TStmpBuff_Out;
                --		 When X"4" => GTPTxStage(0) <= TStmpBuff_Out; TxCRCDat(0) <= TStmpBuff_Out;
                --		 When X"3" => GTPTxStage(0) <= X"FF01"; TxCRCDat(0) <= X"FF01";
                --		 --When X"2" => GTPTxStage(0) <= X"0004"; TxCRCDat(0) <= X"0004";
                --		 When X"0" => GTPTxStage(0) <= X"BC3C"; TxCRCDat(0) <= X"0000";
                --		 When others => GTPTxStage(0) <= X"0000"; TxCRCDat(0) <= X"0000";
                --      end case;	
                --elsif Packet_Former = WrtGRPkt2
                --   then
                --		Case Pkt_Timer is
                --		 When X"A" => GTPTxStage(0) <= X"1C" & TxSeqNo(0) & "0" & X"6"; TxCRCDat(0) <= X"0000"; 
                --		 --When X"9" => GTPTxStage(0) <= CRCErrCnt & LosCounter & "000" & PLLStat;
                --		 --               TxCRCDat(0) <= CRCErrCnt & LosCounter & "000" & PLLStat; -- Status  
                --		 --When X"8" => GTPTxStage(0) <= ExtuBunchCount(15 downto 0); 
                --		 --               TxCRCDat(0) <= ExtuBunchCount(15 downto 0); 
                --		 --When X"7" => GTPTxStage(0) <= ExtuBunchCount(31 downto 16);  
                --		 --               TxCRCDat(0) <= ExtuBunchCount(31 downto 16); 
                --		 --When X"6" => GTPTxStage(0) <= "000" & Beam_On & X"000"; 
                --		 --               TxCRCDat(0) <= "000" & Beam_On & X"000";
                --		 --When X"5" => GTPTxStage(0) <= LastWindow;
                --		 --               TxCRCDat(0) <= LastWindow; -- last window
                                --		 --When X"4" => GTPTxStage(0) <= HeartBtCnt; 
                --		 --               TxCRCDat(0) <= HeartBtCnt; -- EWT counter
                --		 --When X"3" => GTPTxStage(0) <= HeartBeatCnt; 
                --		 --               TxCRCDat(0) <= HeartBeatCnt; -- marker counter
                --		 --When X"2" => GTPTxStage(0) <= InjectionTs; 
                --		 --               TxCRCDat(0) <= InjectionTs; -- injection
                --		 When X"0" => GTPTxStage(0) <= X"BC3C"; TxCRCDat(0) <= X"0000"; 
                --		 When others => GTPTxStage(0) <= X"0000"; TxCRCDat(0) <= X"0000";
                --      end case;	
       
            -- Pad is K28.5 K28.1 pair
            --elsif current_state = Loopback then
            --     GTPTxStage <= X"1C90";
            --else GTPTxStage <= X"BC3C"; TxCRCDat <= X"0000";
            end if;

            if (current_state = WrtCtrlHdrPkt and (Pkt_Timer = 8 or Pkt_Timer = 5 
                or (uBwrt = '1' and (Pkt_Timer = 4 or Pkt_Timer = 3)) -- if uB is written, also read it again
                ---or Pkt_Timer = 7 or Pkt_Timer = 4 or Pkt_Timer = 3 or Pkt_Timer = 2)
                ))
                or (current_state = WrtDatPkt and Pkt_Timer > 2 and EvTxWdCnt > 0)
                    then                              EventBuff_RdEn <= '1';  --Debug(9) <= '1';
                        if EventBuff_Empty = '0' then EventBuffErr <= '0';
                        else                          EventBuffErr <= '1';
                        end if;
            else                                      EventBuff_RdEn <= '0';               
                                                      EventBuffErr <= EventBuffErr;
            end if;

            if (current_state = WrtDCSPkt and (Pkt_Timer = 7 or Pkt_Timer = 6)) --reads are one cycle ahead
                    then DCSBuff_rd_en <= '1';
            else         DCSBuff_rd_en <= '0';
            end if;

            -- Increment the sequence number and clear CRC when sending Packet ID
            if --(UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(0)) or
                 ((current_state = WrtHdrPkt or current_state = WrtHdrPktTimeout or current_state = WrtCtrlHdrPkt or current_state = WrtDCSPkt
                or current_state = WrtDatPkt or current_state = WrtGRPkt or current_state = WrtGRPkt2) and Pkt_Timer = 10)
                    then TxSeqNo <= TxSeqNo + 1;
                         TxCRCRst <= '1';
            else         TxSeqNo <= TxSeqNo;
                         TxCRCRst <= '0';
            end if;

            -- Accumulate CRC while transmitting data
            if -- (UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(2)) or
                 ((  current_state = WrtHdrPkt or current_state = WrtHdrPktTimeout or current_state = WrtCtrlHdrPkt
                or ( current_state = WrtDCSPkt and FormStatReg = "111")
                --or ( current_state = WrtGRPkt  and FormStatReg = "111")
                --or ( current_state = WrtGRPkt2) -- and FormStatReg = "110")
                or   current_state = WrtDatPkt) and Pkt_Timer /= 0 and Pkt_Timer /= 10)
                    then TxCRCEn <= '1';
            else         TxCRCEn <= '0';
            end if;

            -- One byte is control when sending the packet ID
            if --(UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(0)) or
                 ((current_state = WrtHdrPkt or current_state = WrtHdrPktTimeout or current_state = WrtCtrlHdrPkt or current_state = WrtDCSPkt
                or current_state = WrtDatPkt 
                --or current_state = WrtGRPkt or current_state = WrtGRPkt2
                ) and Pkt_Timer = 9)
                    then TxCharIsK_Pipe <= "10";
                    -- Two bytes are data when sending the packet payload
            elsif -- (UsrWRDL(0) = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = GTPWrtAddr(2)) or
                 ((current_state = WrtHdrPkt or current_state = WrtHdrPktTimeout or current_state = WrtCtrlHdrPkt 
                or (	current_state = WrtDCSPkt and FormStatReg = "111") -- neded because it starts with Pkt_Timer = 0, with BC3C
                --or (	current_state = WrtGRPkt and FormStatReg = "111")
                --or (	current_state = WrtGRPkt2) -- and FormStatReg = "110") 
                or current_state = WrtDatPkt) and Pkt_Timer /= 10 and Pkt_Timer /= 9)
                    then TxCharIsK_Pipe <= "00";
            -- Both bytes are K characters when sending pads
            else         TxCharIsK_Pipe <= "11";
            end if;
        
            ---- latch uB status for comparison
            --if Packet_Former = Idle then
            --    uBcheckRef <= X"ffffffff";
            --elsif Packet_Former = WrtHdrPkt then
            --    if    Pkt_Timer = X"6" then
            --        uBcheckRef(31 downto 16) <= TStmpBuff_Out;
            --        uBcheckRef(15 downto  0) <= uBcheckRef(15 downto 0);
            --    elsif Pkt_Timer = X"5" then
            --        uBcheckRef(31 downto 16) <= uBcheckRef(31 downto 16);
            --        uBcheckRef(15 downto  0) <= TStmpBuff_Out;
            --    else
            --        uBcheckRef <= uBcheckRef;
            --    end if;
            --end if;

            if GTPTxBuff_Pipe /= X"BC3C"
            then GTPTxBuff_wr_en <= '1';
            else GTPTxBuff_wr_en <= '0';
            end if;
				
				GTPTxBuff_In <= GTPTxBuff_Pipe;
				GTPTx <= GTPTx_Pipe;
				TxCharIsK <= TxCharIsK_Pipe;

        end if; -- rising edge
    end process state_and_output_process;

end architecture rtl;