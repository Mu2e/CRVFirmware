
 
 
 




window new WaveWindow  -name  "Waves for BMG Example Design"
waveform  using  "Waves for BMG Example Design"


      waveform add -signals /Hist_Ram_tb/status
      waveform add -signals /Hist_Ram_tb/Hist_Ram_synth_inst/bmg_port/RSTA
      waveform add -signals /Hist_Ram_tb/Hist_Ram_synth_inst/bmg_port/CLKA
      waveform add -signals /Hist_Ram_tb/Hist_Ram_synth_inst/bmg_port/ADDRA
      waveform add -signals /Hist_Ram_tb/Hist_Ram_synth_inst/bmg_port/DINA
      waveform add -signals /Hist_Ram_tb/Hist_Ram_synth_inst/bmg_port/WEA
      waveform add -signals /Hist_Ram_tb/Hist_Ram_synth_inst/bmg_port/DOUTA
      waveform add -signals /Hist_Ram_tb/Hist_Ram_synth_inst/bmg_port/RSTB
      waveform add -signals /Hist_Ram_tb/Hist_Ram_synth_inst/bmg_port/CLKB
      waveform add -signals /Hist_Ram_tb/Hist_Ram_synth_inst/bmg_port/ADDRB
      waveform add -signals /Hist_Ram_tb/Hist_Ram_synth_inst/bmg_port/DINB
      waveform add -signals /Hist_Ram_tb/Hist_Ram_synth_inst/bmg_port/WEB
      waveform add -signals /Hist_Ram_tb/Hist_Ram_synth_inst/bmg_port/DOUTB
console submit -using simulator -wait no "run"
