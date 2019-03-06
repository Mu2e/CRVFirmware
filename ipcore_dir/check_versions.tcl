##
## Core Generator Run Script, generator for Project Navigator checkversion command
##

proc findRtfPath { relativePath } {
   set xilenv ""
   if { [info exists ::env(XILINX) ] } {
      if { [info exists ::env(MYXILINX)] } {
         set xilenv [join [list $::env(MYXILINX) $::env(XILINX)] $::xilinx::path_sep ]
      } else {
         set xilenv $::env(XILINX)
      }
   }
   foreach path [ split $xilenv $::xilinx::path_sep ] {
      set fullPath [ file join $path $relativePath ]
      if { [ file exists $fullPath ] } {
         return $fullPath
      }
   }
   return ""
}

source [ findRtfPath "data/projnav/scripts/dpm_cgUtils.tcl" ]

set result [ run_cg_vcheck {C:/Experiments/mu2e/SiPMFE/FPGA/AFE_Fpga_A/ipcore_dir/CMD_Fifo.xco C:/Experiments/mu2e/SiPMFE/FPGA/AFE_Fpga_A/ipcore_dir/LPDDR_Ctrl.xco C:/Experiments/mu2e/SiPMFE/FPGA/AFE_Fpga_A/ipcore_dir/AFE_DP_Pipeline.xco C:/Experiments/mu2e/SiPMFE/FPGA/AFE_Fpga_A/ipcore_dir/DAC_Ram.xco C:/Experiments/mu2e/SiPMFE/FPGA/AFE_Fpga_A/ipcore_dir/SysPLL.xco C:/Experiments/mu2e/SiPMFE/FPGA/AFE_Fpga_A/ipcore_dir/DPRam256x16.xco} xc6slx25-3csg324 ]

if { $result == 0 } {
   puts "Core Generator checkversion command completed successfully."
} elseif { $result == 1 } {
   puts "Core Generator checkversion command failed."
} elseif { $result == 3 || $result == 4 } {
   # convert 'version check' result to real return range, bypassing any messages.
   set result [ expr $result - 3 ]
} else {
   puts "Core Generator checkversion cancelled."
}
exit $result
