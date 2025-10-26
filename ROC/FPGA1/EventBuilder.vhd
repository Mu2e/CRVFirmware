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
        LinkFIFORdCnt   : in  Array_3x14;
        LinkFIFOEmpty   : in  std_logic_vector(2 downto 0);
        LinkFIFORdReq   : out std_logic_vector(2 downto 0);
		  LinkFIFORst     : out std_logic;
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
        uBinHeader      : in  std_logic; -- no longer used, alwasy 1
        uBwrt           : in  std_logic;
        GA              : in  std_logic_vector( 1 downto 0);
		  sendGrCnt       : in  std_logic_vector( 7 downto 0);
		  -- GR data (fake events)
		  ExtuBunchCount  : in  std_logic_vector(47 downto 0);
		  HeartBtCnt      : in  std_logic_vector(15 downto 0);
		  HeartBeatCnt    : in  std_logic_vector(15 downto 0);
		  LastWindow      : in  std_logic_vector(15 downto 0);
		  Stats           : in  std_logic_vector(15 downto 0);
		  InjectionTs     : in  std_logic_vector(15 downto 0);
		  InjectionWindow : in  std_logic_vector(15 downto 0);
		  FakeNum         : out std_logic_vector( 7 downto 0);  -- counts how many events were generated
		  -- debug
		  uBDebugOut      : out std_logic_vector (96 downto 0)
    );
end entity EventBuilder;

architecture rtl of EventBuilder is
    Type Event_Builder_Seq is (Idle,RdInWdCnt0,RdInWdCnt1,RdInWdCnt2,SumWdCnt,CheckWdCnt,WrtWdCnt,--,WrtWdCnt0,WrtWdCnt1,WrtWdCnt2,
	 RdStat0,RdStat1,RdStat2,WrtStat,WrtStat2,WrtUbLow,WrtUbHigh,WaitEvent,ReadFIFO,ReadFIFO0,ReadFIFO1,ReadFIFO2,
	  RduB,RdUbLow,RdUbHigh,VerifyUb,
     Fake,FakeWrite,FakeReset);
    signal current_state, next_state : Event_Builder_Seq;

    -- signal StatOr, Stat0 : std_logic_vector (7 downto 0); 
	 signal StatNew       : std_logic_vector (31 downto 0);
    signal FakeCnt       : std_logic_vector (11 downto 0);
	 signal FakeCntCalls  : std_logic_vector (7 downto 0);
	 signal FakeCntCalls_debug  : std_logic_vector (7 downto 0);
    signal EventSum, FakeDat: std_logic_vector (15 downto 0); 
	 signal Link0, Link1, Link2 : std_logic_vector (15 downto 0);  -- used to precalcualte event counts
    signal uBcheck, uB0, uB1, uB2 : std_logic_vector (31 downto 0);
    signal uBcheckFlag   : std_logic;
	 signal active_flag    : std_logic_vector (2 downto 0);
    signal FIFOCount     : Array_3x16;
	 signal EvBuffWrtGate  : std_logic;
	 
	 -- out buffers
	 signal LinkFIFORdReq_b                      : std_logic_vector(2 downto 0);
	 signal EventBuff_Dat_b,   EventBuff_Dat_reg   : std_logic_vector (15 downto 0);
    signal EventBuff_WrtEn_b, EventBuff_WrtEn_reg : std_logic;
	 
	 -- debug
	 signal debugCnt : std_logic_vector (15 downto 0);
	 signal uBDebug : std_logic_vector (96 downto 0);
	 signal uBDebugReg : std_logic_vector (96 downto 0);

     -- syncs for clock domain transitions
     signal InjectionTs_sync1, InjectionTs_sync2 : std_logic_vector (15 downto 0);

     -- latch data that is async at the start of event
     signal InjectionWindow_latched : std_logic_vector (15 downto 0);
     signal InjectionTs_latched : std_logic_vector (15 downto 0);
	  
	  signal truncate : std_logic; -- flag indicating DAQ limit data truncation
	  signal WordsWrittenTotal : std_logic_vector(15 downto 0);
	  -- DAQ is limited to 2^11 packets, there is one header packet so
	  -- that is 2^11*8-1 = 16384-8 words
	  -- we want to keep full hits (11 words) as they are so 
	  -- 1487 x 11 = 16357 words (0x3fe5)
	  constant MAX_EVENT_SUM : std_logic_vector(15 downto 0) := X"3fe5"; --X"0021"; -- X"3fe5"; -- see above, needs to be <= 2^11*8
	  constant HEADER_WORDS : std_logic_vector(15 downto 0) := X"0005"; -- 5 "header" words
	  signal abort: std_logic;
	  

begin
    LinkFIFORdReq   <= LinkFIFORdReq_b; -- no pipeline
	 EventBuff_Dat    <= EventBuff_Dat_reg;   -- could add a pipeline for EventBuffer
	 EventBuff_WrtEn  <= EventBuff_WrtEn_reg; -- could add a pipeline for EventBuffer
	 --FakeNum <= FakeCntCalls;
	 FakeNum <= FakeCntCalls_debug;
	 
	 uBDebugOut <= uBDebugReg;
	 
    
    -- Next state logic (combinational)
    next_state_process: process(current_state, LinkFIFOEmpty, TStmpWds, FakeCnt, 
                                LinkFIFOOut, LinkFIFORdCnt, FormRst, 
                                FIFOCount, MarkerDelayed(0))
--										  -- removed ActiveReg, FormHold, sendGR, uBinHeader
    begin
        --next_state <= current_state;  -- Default assignment

        Case current_state is
            when Idle => --Debug(10 downto 7) <= X"0";
                if LinkFIFOEmpty /= 7 and FormHold = '0' and TStmpWds >= 3 and sendGR = '0'
                                   then next_state <= WaitEvent;
                elsif sendGR = '1' and MarkerDelayed(0 downto 0) /= 0 then next_state <= Fake;
                else                    next_state <= Idle;
                end if;
            when Fake =>                next_state <= FakeWrite;
            when FakeWrite =>
                if FakeCnt = X"00B" + ("0"&sendGrCnt&"000")  then next_state <= FakeReset; -- 11 = 4 -1 + 8
                else                                next_state <= FakeWrite;
                end if;
            when FakeReset =>
				    next_state <= Idle;
				    --if MarkerDelayed(0 downto 0) /= 0 then
					 --    next_state <= FakeReset;
					 --else
					 --    next_state <= Idle;
					 --end if;
					 --                        next_state <= Idle;   
                --if sendGR = '0' then    next_state <= Idle;
                --else                    next_state <= FakeReset;
                --end if;
            when WaitEvent => --Debug(10 downto 7) <= X"1";
                -- Wait for a complete event to be in all link FIFOs from active ports
                if ((LinkFIFOOut(0)(12 downto 0) <= LinkFIFORdCnt(0) and LinkFIFOEmpty(0) = '0') or active_flag(0) = '0') -- ActiveReg(7 downto 0) = 0)
                   and ((LinkFIFOOut(1)(12 downto 0) <= LinkFIFORdCnt(1) and LinkFIFOEmpty(1) = '0') or active_flag(1) = '0') -- ActiveReg(15 downto 8) = 0)
                   and ((LinkFIFOOut(2)(12 downto 0) <= LinkFIFORdCnt(2) and LinkFIFOEmpty(2) = '0') or active_flag(2) = '0') -- ActiveReg(23 downto 16) = 0) 
                  then
						  if    active_flag(1 downto 0) = "00" then next_state <= RdInWdCnt2; 
						  elsif active_flag(0) = '0'           then next_state <= RdInWdCnt1; 
						  else                                    next_state <= RdInWdCnt0;
                    --if    ActiveReg(15 downto 0) = 0   then next_state <= RdInWdCnt2;
                    --elsif ActiveReg( 7 downto 0) = 0   then next_state <= RdInWdCnt1;
                    --else                                    next_state <= RdInWdCnt0;
                    end if;
                elsif FormRst = '1' then                    next_state <= Idle; 
                else                                        next_state <= WaitEvent;
                end if;
         -- Read in three word counts in order to sum into a controller word count
            when RdInWdCnt0 => --Debug(10 downto 7) <= X"2"; 
				    if    active_flag(2 downto 1) = "00" then next_state <= SumWdCnt;
                elsif active_flag(1) = '0'           then next_state <= RdInWdCnt2;
                else                                     next_state <= RdInWdCnt1;
                end if;
                --if    ActiveReg(23 downto 8) = 0         then next_state <= SumWdCnt;
                --elsif ActiveReg(15 downto 8) = 0         then next_state <= RdInWdCnt2;
                --else                                          next_state <= RdInWdCnt1;
                --end if;
            when RdInWdCnt1 => --Debug(10 downto 7) <= X"3";
				    if active_flag(2) = '0'              then next_state <= SumWdCnt;
                else                                     next_state <= RdInWdCnt2;
                end if;
                --if ActiveReg(23 downto 16) = 0           then next_state <= SumWdCnt;
                --else                                          next_state <= RdInWdCnt2;
                --end if;
            when RdInWdCnt2 => --Debug(10 downto 7) <= X"4";
                                                              next_state <= SumWdCnt;
        -- Subtract 2 from each link word count FIFO to account for the word count and status words
            when SumWdCnt => --Debug(10 downto 7) <= X"5"; 
				                                                  next_state <= CheckWdCnt;
				when CheckWdCnt =>
                                                              next_state <= WrtWdCnt;
        -- Write the controller word count	
            when WrtWdCnt => --Debug(10 downto 7) <= X"6"; 
				    if    active_flag(1 downto 0) = "00"        then next_state <= RdStat2;
                elsif active_flag(0) = '0'                  then next_state <= RdStat1;
                else                                            next_state <= RdStat0;
                end if;
                --if    ActiveReg(15 downto 0) = 0          then next_state <= RdStat2;
                --elsif ActiveReg( 7 downto 0) = 0          then next_state <= RdStat1;
                --else                                          next_state <= RdStat0;
                --end if;  
        -- Read the status from the link FIFOs
            when RdStat0 => --Debug(10 downto 7) <= X"7";
				    if    active_flag(2 downto 1) = "00"          then next_state <= RduB;
                elsif active_flag(1) = '0'                      then next_state <= RdStat2;
                else                                              next_state <= RdStat1;
                end if;
                --if    ActiveReg(23 downto 8) = 0          then next_state <= RduB;
                --elsif ActiveReg(15 downto 8) = 0          then next_state <= RdStat2;
                --else                                           next_state <= RdStat1;
                --end if;
            when RdStat1 => --Debug(10 downto 7) <= X"8"; 
				    if active_flag(2) = '0'                          then next_state <= RduB;
                else                                              next_state <= RdStat2;
                end if;
                --if ActiveReg(23 downto 16) = 0            then next_state <= RduB;
                --else                                           next_state <= RdStat2;
                --end if;
            when RdStat2 => --Debug(10 downto 7) <= X"9"; 
                                                               next_state <= RduB;
        
		      -- Write the "OR" of the status as the controller status word
		      -- read uB from all links in two cycles RdUbLow and RdUbHigh, we need RduB because of the delay in the read
		      -- RduB: set FIFORd
		      -- RduBLow: read uB low which is already loaded, 
		      -- VerifyUb: select which uB will be used for the data header and check for consistency between them
            when RduB => 
			       next_state <= RdUbLow;
			   when RdUbLow => 
			       next_state <= RdUbHigh;
			   when RdUbHigh => 
			       next_state <= VerifyUb;
			   when VerifyUb => 
				    next_state <= WrtStat; 
					 
            when WrtStat => 
					 next_state <= WrtStat2;
				when WrtStat2 =>
				    next_state <= WrtUbLow;
                --if uBwrt = '1' then next_state <= WrtUbLow;
                --else                next_state <= ReadFIFO;
                --end if;
            
            when WrtUbLow => 
                next_state <= WrtUbHigh;
            when WrtUbHigh =>
                next_state <= ReadFIFO;
                 
        -- Skip over any Link that has no data
            when ReadFIFO => -- this step doesn't do anything, we could jump it to speed up things
                if FIFOCount(0) /= 0 and active_flag(0) = '1'                then next_state <= ReadFIFO0;
                elsif FIFOCount(0) = 0 and FIFOCount(1) /= 0 
                    and active_flag(2) = '1'                                 then next_state <= ReadFIFO1; 
                elsif FIFOCount(0) = 0 and FIFOCount(1) = 0 
                    and FIFOCount(2) /= 0 and active_flag(2) = '1'           then next_state <= ReadFIFO2; 
                else                                                             next_state <= Idle;
                end if;
        -- Read the data words from the three link FIFOs in succession
            when ReadFIFO0 => --Debug(10 downto 7) <= X"B";
                if FIFOCount(0) = 1 or FIFOCount(0) = 0 then  
                    -- Skip over any Link that has no data
                    if FIFOCount(1) /= 0 and active_flag(1) = '1'   then next_state <= ReadFIFO1; 
                    elsif FIFOCount(1) = 0 and FIFOCount(2) /= 0 and active_flag(2) = '1' 
                                                                           then next_state <= ReadFIFO2;
                    else                                                        next_state <= Idle;
                    end if;
                elsif (FormRst = '1' or abort = '1')                             then next_state <= Idle;
                else                                                             next_state <= ReadFIFO0;
                end if;
            when ReadFIFO1 => --Debug(10 downto 7) <= X"C";
                if FIFOCount(1) = 1 or FIFOCount(1) = 0 then
                    -- Skip over any Link that has no data
                    if FIFOCount(2) /= 0  and active_flag(2) = '1'  then next_state <= ReadFIFO2;
                    else                                                         next_state <= Idle;
                    end if;
                elsif (FormRst = '1' or abort = '1') then                        next_state <= Idle; 
                else                                                             next_state <= ReadFIFO1;
                end if;
            when ReadFIFO2 => --Debug(10 downto 7) <= X"D";
                if FIFOCount(2) = 1 or FIFOCount(2) = 0                     then next_state <= Idle;
                elsif (FormRst = '1' or abort = '1')                        then  next_state <= Idle; 
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
            current_state <= Idle;
            FakeCnt <= (others => '0');
            FakeDat <= (others => '0');
            EventSum <= (others => '0');
            Link0 <= (others => '0');
            Link1 <= (others => '0');
            Link2 <= (others => '0');
				--EventBuff_Dat     <= (others => '0');
            EventBuff_Dat_b <= (others => '0');
				EventBuff_Dat_reg <= (others => '0');
            -- StatOr <= (others => '0');
            -- Stat0 <= (others => '0');
				StatNew <= (others => '0');
            uBcheck <= (others => '0');
				uB0 <= (others => '0');
				uB1 <= (others => '0');
				uB2 <= (others => '0');
            uBcheckFlag <= '0';
				uBDebugReg <= (others => '0');
            FIFOCount <= (others => (others => '0'));
            LinkFIFORdReq_b <= (others => '0');
				--EventBuff_WrtEn   <= '0';
            EventBuff_WrtEn_b <= '0';
				EventBuff_WrtEn_reg <= '0';
				EvBuffWrtGate <= '0';
				--FakeCntCalls <= (others => '0');
				FakeCntCalls_debug <= (others => '0');
				debugCnt <= (others => '0');
            InjectionTs_sync1 <= (others => '0');
            InjectionTs_sync2 <= (others => '0');
            InjectionWindow_latched <= (others => '0');
            InjectionTs_latched <= (others => '0');
				truncate <= '0';
				WordsWrittenTotal <= (others => '0');
				abort <= '0';
				---first_injection <= '0';
				

        elsif rising_edge(clk) then
            current_state <= next_state;

            -- synchronizations
            InjectionTs_sync1 <= InjectionTs;
            InjectionTs_sync2 <= InjectionTs_sync1;

            if current_state = FakeWrite then
				    if    FakeCnt < X"00B"                         then debugCnt <= debugCnt; -- header
					elsif FakeCnt = X"00B" + ("0"&sendGrCnt&"000") then debugCnt <= debugCnt; -- no inc. in last
					else                                                debugCnt <= debugCnt + 1; -- this counts one too much?
					 
					end if;
                    InjectionWindow_latched <= InjectionWindow;
                    InjectionTs_latched     <= InjectionTs_sync2;
				else                       
                                                                       debugCnt <= debugCnt;
                    InjectionWindow_latched <= InjectionWindow_latched;
                    InjectionTs_latched <= InjectionTs_latched;
				end if;

            if current_state = FakeReset then
					FakeCntCalls_debug <= FakeCntCalls_debug + 1;
				else
					FakeCntCalls_debug <= FakeCntCalls_debug;
				end if;

            if current_state = Fake then
                --FakeCnt <= X"B"; -- 11 = 8+4-1
                FakeDat <= X"0008" + ("0"&sendGrCnt&"000"); -- payload, exclude cnt + 3 x uB number
					 --FakeCntCalls <= FakeCntCalls + 1;
            elsif current_state = FakeWrite then
                --FakeCnt <= FakeCnt - 1;
					 --FakeCntCalls <= FakeCntCalls;
                Case FakeCnt is
                    when X"000" => FakeDat <= ExtuBunchCount(15 downto 0);
                    when X"001" => FakeDat <= ExtuBunchCount(31 downto 16);
                    when X"002" => FakeDat <= ExtuBunchCount(47 downto 32);
                    when X"003" => FakeDat <= X"cafe";
                    when X"004" => FakeDat <= HeartBtCnt;   -- EWT counter
                    when X"005" => FakeDat <= HeartBeatCnt; -- marker counter
                    when X"006" => FakeDat <= LastWindow;
                    when X"007" => FakeDat <= Stats;
                    when X"008" => FakeDat <= InjectionWindow_latched; -- timestamp of injection (from the last window!)
                    when X"009" => FakeDat <= InjectionTs_latched;   -- time of last injection period
                    when X"00a" => FakeDat <= X"beef";
                    when others => FakeDat <= debugCnt;
                 end Case;	 
			  
            else
                --FakeCnt <= FakeCnt;
					 --FakeCntCalls <= FakeCntCalls;
                FakeDat <= (others => '0');
            end if;
				
				if current_state = FakeWrite then FakeCnt <= FakeCnt + 1;
				else                              FakeCnt <= (others => '0');
				end if;

            -- precalcualte the -4
		 	 if current_state = WaitEvent then
				  Link0 <= LinkFIFOOut(0) - 4;
				  Link1 <= LinkFIFOOut(1) - 4;
				  Link2 <= LinkFIFOOut(2) - 4;
			 else
				  Link0 <= Link0;  -- Hold
				  Link1 <= Link1;
				  Link2 <= Link2;
			 end if;

            -- Sum the word counts from the three Link FIFOs.
		    if current_state = Idle then EventSum <= (others => '0');
            -- Account for removing the word count and status words from the data
            elsif current_state = RdInWdCnt0 then 
                --if uBinHeader = '0' then EventSum <= EventSum + LinkFIFOOut(0) - 2; else 
					 EventSum <= EventSum + Link0; -- LinkFIFOOut(0) - 4; -- end if;
            elsif current_state = RdInWdCnt1 then 
                --if uBinHeader = '0' then EventSum <= EventSum + LinkFIFOOut(1) - 2; else 
					 EventSum <= EventSum + Link1; -- LinkFIFOOut(1) - 4; -- end if;
            elsif current_state = RdInWdCnt2 then 
                --if uBinHeader = '0' then EventSum <= EventSum + LinkFIFOOut(2) - 2; else 
					 EventSum <= EventSum + Link2; -- LinkFIFOOut(2) - 4; -- end if;
            elsif current_state = CheckWdCnt then
				    if EventSum > MAX_EVENT_SUM then EventSum <= MAX_EVENT_SUM; else EventSum <= EventSum; end if;
				else EventSum <= EventSum;
            end if;

            -- Sum the word counts from the three Link FIFOs.
		    if current_state = Idle then EventSum <= (others => '0');
            -- Account for removing the word count and status words from the data
            elsif current_state = RdInWdCnt0 then 
                --if uBinHeader = '0' then EventSum <= EventSum + LinkFIFOOut(0) - 2; else 
					 EventSum <= EventSum + Link0; -- LinkFIFOOut(0) - 4; -- end if;
            elsif current_state = RdInWdCnt1 then 
                --if uBinHeader = '0' then EventSum <= EventSum + LinkFIFOOut(1) - 2; else 
					 EventSum <= EventSum + Link1; -- LinkFIFOOut(1) - 4; -- end if;
            elsif current_state = RdInWdCnt2 then 
                --if uBinHeader = '0' then EventSum <= EventSum + LinkFIFOOut(2) - 2; else 
					 EventSum <= EventSum + Link2; -- LinkFIFOOut(2) - 4; -- end if;
            elsif current_state = CheckWdCnt then
				    if EventSum > MAX_EVENT_SUM then EventSum <= MAX_EVENT_SUM; else EventSum <= EventSum; end if;
				else EventSum <= EventSum;
            end if;
				
				if current_state = Idle then
                 truncate <= '0';
				elsif current_state = CheckWdCnt then
				    if EventSum > MAX_EVENT_SUM then 
					     truncate <= '1';
					 else
					     truncate <= '0';
					 end if;
				else
				    truncate <= truncate;
				end if;
            
        -- storte the number of events from the first two FPGAs
        --    if    current_state = Idle       then Event0 <= (others => '0');
        --    elsif current_state = RdInWdCnt0 then Event0 <= LinkFIFOOut(0) - 2;
        --    else                                  Event0 <= Event0;
        --    end if;
        --    if    current_state = Idle       then Event1 <= (others => '0');
        --    elsif current_state = RdInWdCnt1 then Event1 <= LinkFIFOOut(1) - 2;
        --    else                                  Event1 <= Event1;
        --    end if;
        --    if    current_state = Idle       then Event2 <= (others => '0');
        --    elsif current_state = RdInWdCnt2 then Event2 <= LinkFIFOOut(2) - 2;
        --    else                                  Event2 <= Event2;
        --    end if;
        
            -- Select the data source for the event buffer FIFO
            if current_state = WrtWdCnt     then EventBuff_Dat_b <= EventSum;
            --elsif Event_Builder = WrtStat	then EventBuff_Dat <= uBcheckFlag & Stat0(6 downto 0) & StatOR;
            --elsif current_state = WrtStat   then EventBuff_Dat_b <= "0" & Stat0(6 downto 0) & StatOR;
				elsif current_state = WrtStat   then EventBuff_Dat_b <=  StatNew(31) & (uBcheckFlag or StatNew(30)) & StatNew(29 downto 16);
				elsif current_state = WrtStat2  then EventBuff_Dat_b <= StatNew(15 downto  0);
            --elsif current_state = WrtWdCnt0 then EventBuff_Dat_b <= Event0;
            --elsif current_state = WrtWdCnt1 then EventBuff_Dat_b <= Event1;
            --elsif current_state = WrtWdCnt2 then EventBuff_Dat_b <= Event2;
            elsif current_state = WrtUbLow  then EventBuff_Dat_b <= uBcheck(15 downto 0);
            elsif current_state = WrtUbHigh then EventBuff_Dat_b <= uBcheck(31 downto 16);
            elsif LinkFIFORdReq_b(0) = '1'  then EventBuff_Dat_b <= LinkFIFOOut(0);
            elsif LinkFIFORdReq_b(1) = '1'  then EventBuff_Dat_b <= LinkFIFOOut(1);
            elsif LinkFIFORdReq_b(2) = '1'  then EventBuff_Dat_b <= LinkFIFOOut(2);
            elsif current_state = FakeWrite then EventBuff_Dat_b <= FakeDat;
            else                                 EventBuff_Dat_b <= EventBuff_Dat_b;
            end if;
        
            -- Do an "or" of the FEB error words for the cotroller error word
				-- New Format 10/23/2025
				-- 0-23: which FEB reports any problem
				-- 24: any feb uBunch Number mismatch
				-- 25: any feb buffer issue (Empty at Readout Begin, DEAD BEEF match error, 
				-- 26: any feb overflow (OverRun Not Empty at Readout End, and future overflow flags)
				-- 27: group 1 issue
				-- 28: group 2 issue
				-- 29: group 3 issue
				-- 30: any group uB match error
				-- 31: any group truncation
				
			  if current_state = WrtWdCnt then
			               StatNew <= truncate & "000" & x"0000000"; -- reset!
           elsif current_state = RdStat0 then 
                        -- StatOr <= StatOr or LinkFIFOOut(0)(7 downto 0);
                        -- Stat0  <=           LinkFIFOOut(0)(7 downto 0);
								StatNew( 7 downto 0)  <= LinkFIFOOut(0)(7 downto 0); -- Link0, ports
								StatNew(23 downto 8)  <= StatNew(23 downto 8);
								StatNew(26 downto 24) <= LinkFIFOOut(0)(10 downto 8); -- Link0, issue type
								StatNew(27)           <= LinkFIFOOut(0)(12) or LinkFIFOOut(0)(11); -- Link0, ROC
								StatNew(29 downto 28) <= StatNew(29 downto 28);
								StatNew(31 downto 30) <= LinkFIFOOut(0)(12 downto 11); -- Link0, issue type
            elsif current_state = RdStat1 then 
                        --StatOr <= StatOr or LinkFIFOOut(1)(7 downto 0);
                        --Stat0 <= Stat0;
								StatNew( 7 downto 0)  <= StatNew( 7 downto 0);
								StatNew(15 downto 8)  <= LinkFIFOOut(1)(7 downto 0); -- Link1, ports
								StatNew(23 downto 16) <= StatNew(23 downto 16);
								StatNew(26 downto 24) <= StatNew(26 downto 24) or LinkFIFOOut(1)(10 downto 8); -- Link1, issue type
								StatNew(27)           <= StatNew(27);
								StatNew(28)           <= LinkFIFOOut(1)(12) or LinkFIFOOut(1)(11); -- Link1, ROC
								StatNew(29)           <= StatNew(29);
								StatNew(31 downto 30) <= StatNew(31 downto 30) or LinkFIFOOut(1)(12 downto 11); -- Link1, issue type
            elsif current_state = RdStat2 then 
                        --StatOr <= StatOr or LinkFIFOOut(2)(7 downto 0);
                        --Stat0 <= Stat0;
								StatNew(15 downto 0)  <= StatNew(15 downto 0);
								StatNew(23 downto 16) <= LinkFIFOOut(2)(7 downto 0); -- Link2, ports
								StatNew(26 downto 24) <= StatNew(26 downto 24) or LinkFIFOOut(2)(10 downto 8); -- Link2, issue type
								StatNew(28 downto 27) <= StatNew(28 downto 27);
								StatNew(29)           <= LinkFIFOOut(2)(12) or LinkFIFOOut(2)(11); -- Link2, ROC
								StatNew(31 downto 30) <= StatNew(31 downto 30) or LinkFIFOOut(2)(12 downto 11); -- Link2, issue type
        else --StatOr <= StatOr; Stat0 <= Stat0; 
		       StatNew <= StatNew;
        end if;

        -- read uB numbers from input buffers
		  if    current_state = Idle then      
		                                       uB0 <= (others =>'1');
		                                       uB1 <= (others =>'1');
															uB2 <= (others =>'1');
		  elsif current_state = RdUbLow then   
		                                       uB0(31 downto 16) <= uB0(31 downto 16);
                                             uB0(15 downto  0) <= LinkFIFOOut(0);
															uB1(31 downto 16) <= uB1(31 downto 16);
															uB1(15 downto  0) <= LinkFIFOOut(1);
															uB2(31 downto 16) <= uB2(31 downto 16);
															uB2(15 downto  0) <= LinkFIFOOut(2);
		  elsif current_state = RdUbHigh then  
		                                       uB0(31 downto 16) <= LinkFIFOOut(0);
		                                       uB0(15 downto  0) <= uB0(15 downto  0);
															uB1(31 downto 16) <= LinkFIFOOut(1);
		                                       uB1(15 downto  0) <= uB1(15 downto  0);
															uB2(31 downto 16) <= LinkFIFOOut(2);
		                                       uB2(15 downto  0) <= uB2(15 downto  0);
		  else                                 
		                                       uB0 <= uB0;
		                                       uB1 <= uB1;
															uB2 <= uB2;
		  end if;
		  
		  
		  -- precompute active flags
		  if ActiveReg( 7 downto  0)  /= 0 then active_flag(0) <= '1'; else active_flag(0) <= '0'; end if;
		  if ActiveReg(15 downto  8)  /= 0 then active_flag(1) <= '1'; else active_flag(1) <= '0'; end if;
		  if ActiveReg(23 downto 16)  /= 0 then active_flag(2) <= '1'; else active_flag(2) <= '0'; end if;
		  
		  if    current_state = Idle then      uBcheck <= (others =>'1');
		  elsif current_state = VerifyUb then 
		      if    active_flag(0) = '1' then uBcheck <= uB0;
				elsif active_flag(1) = '1' then uBcheck <= uB1;
				else                           uBcheck <= uB2;
				end if;
		  else uBcheck <= uBcheck;
		  end if;
		  
		  if    current_state = Idle then     uBcheckFlag <= '0';
		  elsif current_state = VerifyUb then
			  if     active_flag = "111" then 
					if (uB0 /= uB1) or (uB0 /= uB2)
					                      then   uBcheckFlag <= '1';
					else                         uBcheckFlag <= '0';
					end if;
			  elsif  active_flag = "011" then 
					if (uB1 /= uB0) then         uBcheckFlag <= '1';
					else                         uBcheckFlag <= '0';
					end if;
			  elsif  active_flag = "110" then
					if (uB2 /= uB1)  then        uBcheckFlag <= '1';
					else                         uBcheckFlag <= '0';
					end if;
			  elsif  active_flag = "101" then
					if (uB2 /= uB0) then         uBcheckFlag <= '1';
					else                         uBcheckFlag <= '0';
					end if;
			   else 
					uBcheckFlag <= '0';
			   end if;
		  else
		    uBcheckFlag <= uBcheckFlag;
	     end if;
				
		  
		  if current_state = WrtStat then 
		      uBDebugReg <= uBcheckFlag & uB0 & uB1 & uB2;
		  else 
		     uBDebugReg <= uBDebugReg;
		  end if;

    -- Count down the words read from each of the link FIFOs
    if current_state = RdInWdCnt0 then 
        --if uBinHeader = '0'                               then FIFOCount(0) <= LinkFIFOOut(0) - 2; 
        --else                                                   
		  FIFOCount(0) <= LinkFIFOOut(0) - 4; 
        --end if;   
    elsif current_state = ReadFIFO0 and FIFOCount(0) /= 0 then FIFOCount(0) <= FIFOCount(0) - 1;
    else                                                       FIFOCount(0) <= FIFOCount(0);
    end if;

    if current_state = RdInWdCnt1 then 
        --if uBinHeader = '0'                               then FIFOCount(1) <= LinkFIFOOut(1) - 2; 
        --else                                                   
		  FIFOCount(1) <= LinkFIFOOut(1) - 4; 
        --end if;
    elsif current_state = ReadFIFO1 and FIFOCount(1) /= 0 then FIFOCount(1) <= FIFOCount(1) - 1;
    else                                                       FIFOCount(1) <= FIFOCount(1);
    end if;

    if current_state = RdInWdCnt2 then 
        --if uBinHeader = '0'                               then FIFOCount(2) <= LinkFIFOOut(2) - 2; 
        --else                                                   
		  FIFOCount(2) <= LinkFIFOOut(2) - 4; 
        --end if;
    elsif current_state = ReadFIFO2 and FIFOCount(2) /= 0 then FIFOCount(2) <= FIFOCount(2) - 1;
    else                                                       FIFOCount(2) <= FIFOCount(2);
    end if;

    -- Link FIFO reads
    -- Microcontroller read
    if (LinkRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkRdAddr(0))
       -- Read of header words, read of data words
       or current_state = RdInWdCnt0 or current_state = RdStat0 or current_state = ReadFIFO0
       --or (current_state = RdUb and uBinHeader = '1') 
		 or current_state = RdUb
		 or current_state = RdUbLow
    then LinkFIFORdReq_b(0) <= '1'; 
    else LinkFIFORdReq_b(0) <= '0'; 
    end if;

    if (LinkRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkRdAddr(1))
    -- Read of header words, read of data words
        or current_state = RdInWdCnt1 or current_state = RdStat1 or current_state = ReadFIFO1
        --or (current_state = RdUb and uBinHeader = '1') 
		  or current_state = RdUb
		  or current_state = RdUbLow
    then LinkFIFORdReq_b(1) <= '1'; 
    else LinkFIFORdReq_b(1) <= '0'; 
    end if;

    if (LinkRDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkRdAddr(2))
    -- Read of header words, read of data words
        or current_state = RdInWdCnt2 or current_state = RdStat2 or current_state = ReadFIFO2
        --or (current_state = RdUb and uBinHeader = '1') 
		  or current_state = RdUb
		  or current_state = RdUbLow
    then LinkFIFORdReq_b(2) <= '1'; 
    else LinkFIFORdReq_b(2) <= '0'; 
    end if;

    if current_state = Idle        then EvBuffWrtGate <= '0';
    elsif current_state = WrtStat2 then EvBuffWrtGate <= '1';
    else                               EvBuffWrtGate <= EvBuffWrtGate;
    end if;

    if current_state = Idle then
        WordsWrittenTotal <= (others => '0');
    elsif EventBuff_WrtEn_b = '1' then
        WordsWrittenTotal <= WordsWrittenTotal + 1;
    else
        WordsWrittenTotal <= WordsWrittenTotal;
    end if;

    if (current_state = WrtWdCnt -- or current_state = WrtWdCnt0 or current_state = WrtWdCnt1 or current_state = WrtWdCnt2
       or current_state = WrtStat or current_state = WrtStat2 or current_state = WrtUbLow or current_state = WrtUbHigh
       or (LinkFIFORdReq_b /= 0 and EvBuffWrtGate = '1')
       or current_state = FakeWrite)
		 and (WordsWrittenTotal < (EventSum + HEADER_WORDS - 1))
    then EventBuff_WrtEn_b <= '1'; --Debug(6) <= '1';
    else EventBuff_WrtEn_b <= '0';  --Debug(6) <= '0';
    end if;
	 
	 if WordsWrittenTotal >= (EventSum + HEADER_WORDS - 1) and EvBuffWrtGate = '1' then
        abort <= '1';
    else
        abort <= '0';
    end if;
	 
	 	 
	 -- pipeline for EventBuffer, might help with timing constraints?
	 EventBuff_Dat_reg    <= EventBuff_Dat_b;   -- could add a pipeline for EventBuffer
	 EventBuff_WrtEn_reg  <= EventBuff_WrtEn_b; -- could add a pipeline for EventBuffer
	 LinkFIFORst <= abort;

end if; -- rising edge
end process state_and_output_process;

end architecture rtl;