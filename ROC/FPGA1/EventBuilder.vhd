LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;

Library UNISIM;
use UNISIM.vcomponents.all;

use work.Project_defs.all;

entity EventBuilder is
    port (
        clk             : in  std_logic; 
        reset           : in  std_logic; -- active high
        FormRst         : in  std_logic;
        -- LinkFIFOs
        LinkFIFOOut     : in  Array_3x16;
        LinkFIFORdCnt   : in  Array_3x13;
        LinkFIFOEmpty   : in  std_logic_vector(2 downto 0);
        LinkFIFORdReq   : out std_logic_vector(2 downto 0);
        -- EventBuffer
        EventBuff_Dat    : out std_logic_vector (15 downto 0);
        EventBuff_WrtEn  : out std_logic;
        -- other signals
        TStmpWds        : in  std_logic_vector( 8 downto 0); -- used to sync with data requests, needed? 
        ActiveReg       : in  std_logic_vector(23 downto 0);
		  MarkerDelayed   : in  std_logic_vector( 3 downto 0); -- used for GR/Fake data trigger
        -- uC interface
        LinkRDDL        : in  std_logic_vector (1 downto 0);
        AddrReg         : in  std_logic_vector(11 downto 0);
        -- Settings
        FormHold        : in  std_logic; -- like and inverted enable, if hight -> hold  
        sendGR          : in  std_logic;
        uBinHeader      : in  std_logic;
        uBwrt           : in  std_logic;
        GA              : in  std_logic_vector( 1 downto 0);
		  -- GR data (fake events)
		  ExtuBunchCount  : in  std_logic_vector(47 downto 0);
		  HeartBtCnt      : in  std_logic_vector(15 downto 0);
		  HeartBeatCnt    : in  std_logic_vector(15 downto 0);
		  LastWindow      : in  std_logic_vector(15 downto 0);
		  Stats           : in  std_logic_vector(15 downto 0);
		  InjectionTs     : in  std_logic_vector(15 downto 0);
		  InjectionWindow : in  std_logic_vector(15 downto 0);
		  FakeNum         : out std_logic_vector( 7 downto 0)  -- counts how many events were generated
    );
end entity EventBuilder;

architecture rtl of EventBuilder is
    Type Event_Builder_Seq is (Idle,RdInWdCnt0,RdInWdCnt1,RdInWdCnt2,SumWdCnt,WrtWdCnt,WrtWdCnt0,WrtWdCnt1,WrtWdCnt2,RdStat0,
    RdStat1,RdStat2,WrtStat,WrtUbLow,WrtUbHigh,WaitEvent,ReadFIFO,ReadFIFO0,ReadFIFO1,ReadFIFO2,
     RdUb0low,RdUb0high,RdUb1low,RdUb1high,RdUb2low,RdUb2high, RduB,
     VerifyUb0low,VerifyUb0high,VerifyUb1low,VerifyUb1high,VerifyUb2low,VerifyUb2high,
     Fake,FakeWrite,FakeReset);
    signal current_state, next_state : Event_Builder_Seq;

    signal StatOr, Stat0 : std_logic_vector (7 downto 0); 
    signal FakeCnt       : std_logic_vector (3 downto 0);
	 signal FakeCntCalls  : std_logic_vector (7 downto 0);
    signal EventSum, Event0, Event1, Event2, FakeDat: std_logic_vector (15 downto 0); 
    signal uBcheck       : std_logic_vector (31 downto 0);
    signal uBcheckFlag   : std_logic;
    signal FIFOCount     : Array_3x16;
	 signal EvBuffWrtGate  : std_logic;
	 
	 -- out buffers
	 signal LinkFIFORdReq_b  : std_logic_vector(2 downto 0);
	 signal EventBuff_Dat_b   : std_logic_vector (15 downto 0);
    signal EventBuff_WrtEn_b : std_logic;
begin
    LinkFIFORdReq   <= LinkFIFORdReq_b; -- no pipeline
	 --EventBuff_Dat    <= EventBuff_Dat_b;   -- could add a pipeline for EventBuffer
	 --EventBuff_WrtEn  <= EventBuff_WrtEn_b; -- could add a pipeline for EventBuffer
	 FakeNum <= FakeCntCalls;
	 
    
    -- Next state logic (combinational)
    next_state_process: process(current_state, LinkFIFOEmpty, FormHold, TStmpWds, sendGR, FakeCnt, 
                                LinkFIFOOut, LinkFIFORdCnt, ActiveReg, FormRst, 
                                uBinHeader, FIFOCount)
    begin
        --next_state <= current_state;  -- Default assignment

        Case current_state is
            when Idle => --Debug(10 downto 7) <= X"0";
                if LinkFIFOEmpty /= 7 and FormHold = '0' and TStmpWds >= 3 and sendGR = '0'
                                   then next_state <= WaitEvent;
                elsif sendGR = '1' and MarkerDelayed /= 0 then next_state <= Fake;
                else                    next_state <= Idle;
                end if;
            when Fake =>                next_state <= FakeWrite;
            when FakeWrite =>
                if FakeCnt /= 0    then next_state <= FakeWrite;
                else                    next_state <= FakeReset;
                end if;
            when FakeReset =>
				                            next_state <= Idle;   
                --if sendGR = '0' then    next_state <= Idle;
                --else                    next_state <= FakeReset;
                --end if;
            when WaitEvent => --Debug(10 downto 7) <= X"1";
                -- Wait for a complete event to be in all link FIFOs from active ports
                if ((LinkFIFOOut(0)(12 downto 0) <= LinkFIFORdCnt(0) and LinkFIFOEmpty(0) = '0') or ActiveReg(7 downto 0) = 0)
                   and ((LinkFIFOOut(1)(12 downto 0) <= LinkFIFORdCnt(1) and LinkFIFOEmpty(1) = '0') or ActiveReg(15 downto 8) = 0)
                   and ((LinkFIFOOut(2)(12 downto 0) <= LinkFIFORdCnt(2) and LinkFIFOEmpty(2) = '0') or ActiveReg(23 downto 16) = 0) 
                  then
                    if    ActiveReg(15 downto 0) = 0   then next_state <= RdInWdCnt2;
                    elsif ActiveReg( 7 downto 0) = 0   then next_state <= RdInWdCnt1;
                    else                                    next_state <= RdInWdCnt0;
                    end if;
                elsif FormRst = '1' then                    next_state <= Idle; 
                else                                        next_state <= WaitEvent;
                end if;
         -- Read in three word counts in order to sum into a controller word count
            when RdInWdCnt0 => --Debug(10 downto 7) <= X"2"; 
                if    ActiveReg(23 downto 8) = 0         then next_state <= SumWdCnt;
                elsif ActiveReg(15 downto 8) = 0         then next_state <= RdInWdCnt2;
                else                                          next_state <= RdInWdCnt1;
                end if;
            when RdInWdCnt1 => --Debug(10 downto 7) <= X"3";
                if ActiveReg(23 downto 16) = 0           then next_state <= SumWdCnt;
                else                                          next_state <= RdInWdCnt2;
                end if;
            when RdInWdCnt2 => --Debug(10 downto 7) <= X"4";
                                                              next_state <= SumWdCnt;
        -- Subtract 2 from each link word count FIFO to account for the word count and status words
            when SumWdCnt => --Debug(10 downto 7) <= X"5"; 
                                                              next_state <= WrtWdCnt;
        -- Write the controller word count	
            when WrtWdCnt => --Debug(10 downto 7) <= X"6"; 
                if    ActiveReg(15 downto 0) = 0          then next_state <= RdStat2;
                elsif ActiveReg( 7 downto 0) = 0          then next_state <= RdStat1;
                else                                          next_state <= RdStat0;
                end if;  
        -- Read the status from the link FIFOs
            when RdStat0 => --Debug(10 downto 7) <= X"7";
                if    ActiveReg(23 downto 8) = 0          then next_state <= RduB;
                elsif ActiveReg(15 downto 8) = 0          then next_state <= RdStat2;
                else                                           next_state <= RdStat1;
                end if;
            when RdStat1 => --Debug(10 downto 7) <= X"8"; 
                if ActiveReg(23 downto 16) = 0            then next_state <= RduB;
                else                                           next_state <= RdStat2;
                end if;
            when RdStat2 => --Debug(10 downto 7) <= X"9"; 
                                                               next_state <= RduB;
        -- Write the "OR" of the status as the controller status word
        
           when RduB => -- this step could be jumped
                if uBinHeader = '0'                        then next_state <= WrtStat;
                else
                    if    ActiveReg(15 downto 0) = 0       then next_state <= RduB2low; -- only FPGA2 active 
                    elsif ActiveReg( 7 downto 0) = 0       then next_state <= RduB1low; -- FPGA1 active, FPGA0 not active, FPGA2 maybe active
                    else                                        next_state <= RduB0low; -- FPGA0 active
                    end if;  
                end if;
        
            when RduB2low =>
                next_state <= RduB2high;
            when RduB2high =>
                next_state <= WrtStat;
            when RduB1low =>
                next_state <= RduB1high;
            when RduB1high =>
                if ActiveReg(23 downto 16) = 0                then next_state <= WrtStat; -- only FPGA1 active
                else                                               next_state <= VerifyuB2low; -- verify against FPGA2, FPGA0 is not active
                end if;
                 
            when VerifyuB2low =>
                next_state <= VerifyuB2high;
            when VerifyuB2high =>
                next_state <= WrtStat;     
            when RduB0low =>
                 next_state <= RduB0high;
            when RduB0high =>
                if    ActiveReg(23 downto 8) = 0 then next_state <= WrtStat; -- only FPGA0
                elsif ActiveReg(15 downto 8) = 0 then next_state <= VerifyuB2low; -- only FPGA2, check against it
                else                                  next_state <= VerifyuB1low; -- verify against FPGA1 and FPGA2
                end if;
                 
            when VerifyuB1low =>
                 next_state <= VerifyuB1high;
            when VerifyuB1high =>
                if ActiveReg(23 downto 16) = 0 then next_state <= WrtStat; -- only FPGA2 not active, done
                 else                               next_state <= VerifyuB2low; -- verify against FPGA2, FPGA0 is not active
                 end if;
        
            when WrtStat => 
                if uBwrt = '1' then next_state <= WrtUbLow;
                else                next_state <= ReadFIFO;
                end if;
            
            when WrtUbLow => 
                next_state <= WrtUbHigh;
            when WrtUbHigh =>
                next_state <= ReadFIFO;
                 
        -- Skip over any Link that has no data
            when ReadFIFO => -- this step doesn't do anything, we could jump it to speed up things
                if FIFOCount(0) /= 0 and ActiveReg(7 downto 0) /= 0         then next_state <= ReadFIFO0;
                elsif FIFOCount(0) = 0 and FIFOCount(1) /= 0 
                    and ActiveReg(15 downto 8) /= 0                         then next_state <= ReadFIFO1; 
                elsif FIFOCount(0) = 0 and FIFOCount(1) = 0 
                    and FIFOCount(2) /= 0 and ActiveReg(23 downto 16) /= 0  then next_state <= ReadFIFO2; 
                else                                                              next_state <= Idle;
                end if;
        -- Read the data words from the three link FIFOs in succession
            when ReadFIFO0 => --Debug(10 downto 7) <= X"B";
                if FIFOCount(0) = 1 or FIFOCount(0) = 0 then  
                    -- Skip over any Link that has no data
                    if FIFOCount(1) /= 0 and ActiveReg(15 downto 8) /= 0   then next_state <= ReadFIFO1; 
                    elsif FIFOCount(1) = 0 and FIFOCount(2) /= 0 and ActiveReg(23 downto 16) /= 0 
                                                                           then next_state <= ReadFIFO2;
                    else                                                        next_state <= Idle;
                    end if;
                elsif FormRst = '1'                                        then next_state <= Idle;
                else                                                             next_state <= ReadFIFO0;
                end if;
            when ReadFIFO1 => --Debug(10 downto 7) <= X"C";
                if FIFOCount(1) = 1 or FIFOCount(1) = 0 then
                    -- Skip over any Link that has no data
                    if FIFOCount(2) /= 0  and ActiveReg(23 downto 16) /= 0  then next_state <= ReadFIFO2;
                    else                                                         next_state <= Idle;
                    end if;
                elsif FormRst = '1' then                                         next_state <= Idle; 
                else                                                             next_state <= ReadFIFO1;
                end if;
            when ReadFIFO2 => --Debug(10 downto 7) <= X"D";
                if FIFOCount(2) = 1 or FIFOCount(2) = 0                     then next_state <= Idle;
                elsif FormRst = '1'                                        then  next_state <= Idle; 
                else                                                             next_state <= ReadFIFO2;
                end if;
            when others => --Debug(10 downto 7) <= X"E";
                                                                                 next_state <= Idle;
          end case;
    end process next_state_process;

    -- State register and output logic (sequential)
    state_and_output_process: process(clk, reset)
    begin
        if reset = '1' then
            current_state <= idle;
            FakeCnt <= (others => '0');
            FakeDat <= (others => '0');
            EventSum <= (others => '0');
            Event0 <= (others => '0');
            Event1 <= (others => '0');
            Event2 <= (others => '0');
            EventBuff_Dat_b <= (others => '0');
            StatOr <= (others => '0');
            Stat0 <= (others => '0');
            uBcheck <= (others => '0');
            uBcheckFlag <= '0';
            FIFOCount <= (others => (others => '0'));
            LinkFIFORdReq_b <= (others => '0');
            EventBuff_WrtEn_b <= '0';
				EvBuffWrtGate <= '0';
				FakeCntCalls <= (others => '0');

        elsif rising_edge(clk) then
            current_state <= next_state;

            if current_state = Fake then
                FakeCnt <= X"b"; -- 11 = 8+4-1
                FakeDat <= X"0008"; -- payload, exclude cnt + 3 x uB number
					 FakeCntCalls <= FakeCntCalls + 1;
            elsif current_state = FakeWrite then
                FakeCnt <= FakeCnt - 1;
					 FakeCntCalls <= FakeCntCalls;
                Case FakeCnt is
                    when X"B" => FakeDat <= ExtuBunchCount(15 downto 0);
                    when X"A" => FakeDat <= ExtuBunchCount(31 downto 16);
                    when X"9" => FakeDat <= ExtuBunchCount(47 downto 32);
                    when X"8" => FakeDat <= X"cafe";
                    when X"7" => FakeDat <= HeartBtCnt;   -- EWT counter
                    when X"6" => FakeDat <= HeartBeatCnt; -- marker counter
                    when X"5" => FakeDat <= LastWindow;
                    when X"4" => FakeDat <= Stats;
                    when X"3" => FakeDat <= InjectionTs;     -- timestamp of injection (from the last window!)
                    when X"2" => FakeDat <= InjectionWindow; -- time of last injection period
                    when X"1" => FakeDat <= X"beef";
                    when others => FakeDat <= (others => '0');
                 end Case;	  
            else
                FakeCnt <= FakeCnt;
                FakeDat <= (others => '0');
            end if;

            -- Sum the word counts from the three Link FIFOs.
		    if current_state = Idle then EventSum <= (others => '0');
            -- Account for removing the word count and status words from the data
            elsif current_state = RdInWdCnt0 then 
                if uBinHeader = '0' then EventSum <= LinkFIFOOut(0) - 2; else EventSum <= LinkFIFOOut(0) - 4; end if;
            elsif current_state = RdInWdCnt1 then 
                if uBinHeader = '0' then EventSum <= LinkFIFOOut(1) - 2; else EventSum <= LinkFIFOOut(1) - 4; end if;
            elsif current_state = RdInWdCnt2 then 
                if uBinHeader = '0' then EventSum <= LinkFIFOOut(2) - 2; else EventSum <= LinkFIFOOut(2) - 4; end if;
            else EventSum <= EventSum;
            end if;
            
        -- storte the number of events from the first two FPGAs
            if    current_state = Idle       then Event0 <= (others => '0');
            elsif current_state = RdInWdCnt0 then Event0 <= LinkFIFOOut(0) - 2;
            else                                  Event0 <= Event0;
            end if;
            if    current_state = Idle       then Event1 <= (others => '0');
            elsif current_state = RdInWdCnt1 then Event1 <= LinkFIFOOut(1) - 2;
            else                                  Event1 <= Event1;
            end if;
            if    current_state = Idle       then Event2 <= (others => '0');
            elsif current_state = RdInWdCnt2 then Event2 <= LinkFIFOOut(2) - 2;
            else                                  Event2 <= Event2;
            end if;
        
            -- Select the data source for the event buffer FIFO
            if current_state = WrtWdCnt     then EventBuff_Dat_b <= EventSum;
            --elsif Event_Builder = WrtStat	then EventBuff_Dat <= uBcheckFlag & Stat0(6 downto 0) & StatOR;
            elsif current_state = WrtStat   then EventBuff_Dat_b <= "0" & Stat0(6 downto 0) & StatOR;
            elsif current_state = WrtWdCnt0 then EventBuff_Dat_b <= Event0;
            elsif current_state = WrtWdCnt1 then EventBuff_Dat_b <= Event1;
            elsif current_state = WrtWdCnt2 then EventBuff_Dat_b <= Event2;
            elsif current_state = WrtUbLow  then EventBuff_Dat_b <= uBcheck(15 downto 0);
            elsif current_state = WrtUbHigh then EventBuff_Dat_b <= uBcheck(31 downto 16);
            elsif LinkFIFORdReq_b(0) = '1'  then EventBuff_Dat_b <= LinkFIFOOut(0);
            elsif LinkFIFORdReq_b(1) = '1'  then EventBuff_Dat_b <= LinkFIFOOut(1);
            elsif LinkFIFORdReq_b(2) = '1'  then EventBuff_Dat_b <= LinkFIFOOut(2);
            elsif current_state = FakeWrite then EventBuff_Dat_b <= FakeDat;
            else                                 EventBuff_Dat_b <= EventBuff_Dat_b;
            end if;
        
            -- Do an "or" of the FEB error words for the cotroller error word
           if current_state = RdStat0 then 
                        StatOr <= StatOr or LinkFIFOOut(0)(7 downto 0);
                        Stat0  <=           LinkFIFOOut(0)(7 downto 0);
            elsif current_state = RdStat1 then 
                        StatOr <= StatOr or LinkFIFOOut(1)(7 downto 0);
                        Stat0 <= Stat0;
            elsif current_state = RdStat2 then 
                        StatOr <= StatOr or LinkFIFOOut(2)(7 downto 0);
                        Stat0 <= Stat0;
        else StatOr <= StatOr; Stat0 <= Stat0;
        end if;

        -- read uB numbers from input buffers
        if    current_state = Idle then      uBcheck <= (others =>'1');
        elsif current_state = RdUb0low then  uBcheck(31 downto 16) <= uBcheck(31 downto 16);
                                             uBcheck(15 downto  0) <= LinkFIFOOut(0);
        elsif current_state = RdUb0high then uBcheck(31 downto  16) <= LinkFIFOOut(0);
                                             uBcheck(15 downto  0) <= uBcheck(15 downto 0);
        elsif current_state = RdUb1low then  uBcheck(31 downto 16) <= uBcheck(31 downto 16);
                                             uBcheck(15 downto  0) <= LinkFIFOOut(1);
        elsif current_state = RdUb1high then uBcheck(31 downto  16) <= LinkFIFOOut(1);
                                             uBcheck(15 downto  0) <= uBcheck(15 downto 0);
        elsif current_state = RdUb2low then  uBcheck(31 downto 16) <= uBcheck(31 downto 16);
                                             uBcheck(15 downto  0) <= LinkFIFOOut(2);
        elsif current_state = RdUb2high then uBcheck(31 downto  16) <= LinkFIFOOut(2);
                                             uBcheck(15 downto  0) <= uBcheck(15 downto 0);
        else                                 uBcheck <= uBcheck;
        end if;

        -- check if uB are consistent
        if current_state = Idle then uBcheckFlag <= '0';
        elsif current_state = VerifyUb0low then
            if (uBcheck(15 downto  0) xor LinkFIFOOut(0)) = X"0000"  then uBcheckFlag <= uBcheckFlag;
            else                                                          uBcheckFlag <= '1';
            end if;
        elsif current_state = VerifyUb0high then
            if (uBcheck(31 downto  16) xor LinkFIFOOut(0)) = X"0000" then uBcheckFlag <= uBcheckFlag;
            else                                                          uBcheckFlag <= '1';
        end if;
        elsif current_state = VerifyUb1low then
            if (uBcheck(15 downto  0) xor LinkFIFOOut(1)) = X"0000" then uBcheckFlag <= uBcheckFlag;
            else                                                         uBcheckFlag <= '1';
            end if;
        elsif current_state = VerifyUb1high then
            if (uBcheck(31 downto  16) xor LinkFIFOOut(1)) = X"0000" then uBcheckFlag <= uBcheckFlag;
            else                                                          uBcheckFlag <= '1';
            end if;
        elsif current_state = VerifyUb2low then
            if (uBcheck(15 downto  0) xor LinkFIFOOut(2)) = X"0000" then uBcheckFlag <= uBcheckFlag;
            else                                                         uBcheckFlag <= '1';
            end if;
        elsif current_state = VerifyUb2high then
            if (uBcheck(31 downto  16) xor LinkFIFOOut(2)) = X"0000" then uBcheckFlag <= uBcheckFlag;
            else                                                          uBcheckFlag <= '1';
        end if;
    else                                                                  uBcheckFlag <= uBcheckFlag;
    end if;

    -- Count down the words read from each of the link FIFOs
    if current_state = RdInWdCnt0 then 
        if uBinHeader = '0'                               then FIFOCount(0) <= LinkFIFOOut(0) - 2; 
        else                                                   FIFOCount(0) <= LinkFIFOOut(0) - 4; 
        end if;   
    elsif current_state = ReadFIFO0 and FIFOCount(0) /= 0 then FIFOCount(0) <= FIFOCount(0) - 1;
    else                                                       FIFOCount(0) <= FIFOCount(0);
    end if;

    if current_state = RdInWdCnt1 then 
        if uBinHeader = '0'                               then FIFOCount(1) <= LinkFIFOOut(1) - 2; 
        else                                                   FIFOCount(1) <= LinkFIFOOut(1) - 4; 
        end if;
    elsif current_state = ReadFIFO1 and FIFOCount(1) /= 0 then FIFOCount(1) <= FIFOCount(1) - 1;
    else                                                       FIFOCount(1) <= FIFOCount(1);
    end if;

    if current_state = RdInWdCnt2 then 
        if uBinHeader = '0'                               then FIFOCount(2) <= LinkFIFOOut(2) - 2; 
        else                                                   FIFOCount(2) <= LinkFIFOOut(2) - 4; 
        end if;
    elsif current_state = ReadFIFO2 and FIFOCount(2) /= 0 then FIFOCount(2) <= FIFOCount(2) - 1;
    else                                                       FIFOCount(2) <= FIFOCount(2);
    end if;

    -- Link FIFO reads
    -- Microcontroller read
    if (LinkRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkRdAddr(0))
       -- Read of header words, read of data words
       or current_state = RdInWdCnt0 or current_state = RdStat0 or current_state = ReadFIFO0
       or (current_state = RdUb and uBinHeader = '1') or current_state = RdUb0low --or current_state = RdUb0high
       or current_state = VerifyUb0low --or current_state = VerifyUb0high
    then LinkFIFORdReq_b(0) <= '1'; 
    else LinkFIFORdReq_b(0) <= '0'; 
    end if;

    if (LinkRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkRdAddr(1))
    -- Read of header words, read of data words
        or current_state = RdInWdCnt1 or current_state = RdStat1 or current_state = ReadFIFO1
        or (current_state = RdUb and uBinHeader = '1') or current_state = RdUb1low --or Event_Builder = RdUb1high
        or current_state = VerifyUb1low --or Event_Builder = VerifyUb1high
    then LinkFIFORdReq_b(1) <= '1'; 
    else LinkFIFORdReq_b(1) <= '0'; 
    end if;

    if (LinkRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkRdAddr(2))
    -- Read of header words, read of data words
        or current_state = RdInWdCnt2 or current_state = RdStat2 or current_state = ReadFIFO2
        or (current_state = RdUb and uBinHeader = '1') or current_state = RdUb2low --or Event_Builder = RdUb2high
        or current_state = VerifyUb2low --or Event_Builder = VerifyUb2high
    then LinkFIFORdReq_b(2) <= '1'; 
    else LinkFIFORdReq_b(2) <= '0'; 
    end if;

    if current_state = Idle       then EvBuffWrtGate <= '0';
    elsif current_state = WrtStat then EvBuffWrtGate <= '1';
    else                               EvBuffWrtGate <= EvBuffWrtGate;
    end if;

    if current_state = WrtWdCnt or current_state = WrtWdCnt0 or current_state = WrtWdCnt1 or current_state = WrtWdCnt2
       or current_state = WrtStat or current_state = WrtUbLow or current_state = WrtUbHigh
       or (LinkFIFORdReq_b /= 0 and EvBuffWrtGate = '1')
       or current_state = FakeWrite
    then EventBuff_WrtEn_b <= '1'; --Debug(6) <= '1';
    else EventBuff_WrtEn_b <= '0';  --Debug(6) <= '0';
    end if;
	 	 
	 -- pipeline for EventBuffer, might help with timing constraints?
	 EventBuff_Dat    <= EventBuff_Dat_b;   -- could add a pipeline for EventBuffer
	 EventBuff_WrtEn  <= EventBuff_WrtEn_b; -- could add a pipeline for EventBuffer


end if; -- rising edge
end process state_and_output_process;

end architecture rtl;