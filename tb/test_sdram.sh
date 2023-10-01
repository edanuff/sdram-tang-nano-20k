#!/bin/sh
rm sdram.vvp
rm sdram.log
rm sdram_testbench.vcd
iverilog -g2012 -o sdram.vvp mt48lc2m32b2.v sdram.v test_sdram_tb.sv
vvp -lsdram.log sdram.vvp
