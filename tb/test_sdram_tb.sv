`timescale 1ns / 1ps
module tb_Sdram;

    // Clock signal
    reg clk;
    reg resetn;

    // Signals for the multiple ports
    reg rd;
    reg wr;
    reg [22:0] addr;
    reg [7:0] din;
    wire [7:0] dout;
    wire [15:0] dout16;
    wire [31:0] dout32;
    wire data_ready;
    wire busy;
    reg refresh;

    wire [31 : 0] SDRAM_DQ_i; // SDRAM I/O
    wire [31 : 0] SDRAM_DQ_o; // SDRAM I/O
    wire [10 : 0] SDRAM_A; // SDRAM Address
    wire [1 : 0] SDRAM_BA; // Bank Address
    wire SDRAM_CLK; // Clock
    wire SDRAM_CKE; // Synchronous Clock Enable
    wire SDRAM_nCS; // CS#
    wire SDRAM_nRAS; // RAS#
    wire SDRAM_nCAS; // CAS#
    wire SDRAM_nWE; // WE#
    wire [3 : 0] SDRAM_DQM; // I/O Mask
    wire SDRAM_CMD = {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE};

   // VCD Dumping
    initial begin
        $dumpfile("sdram_testbench.vcd");
        $dumpvars(0, tb_Sdram);
    end

    mt48lc2m32b2 sdram (
        .Dq(SDRAM_DQ_i), 
        .Dq_o(SDRAM_DQ_o), 
        .Addr(SDRAM_A), 
        .Ba(SDRAM_BA), 
        .Clk(SDRAM_CLK), 
        .Cke(SDRAM_CKE), 
        .Cs_n(SDRAM_nCS), 
        .Ras_n(SDRAM_nRAS), 
        .Cas_n(SDRAM_nCAS), 
        .We_n(SDRAM_nWE), 
        .Dqm(SDRAM_DQM)
    );

    sdram controller (
        .clk(clk), 
        .clk_sdram(~clk), 
        .resetn(resetn),

        .rd(rd), 
        .wr(wr), 
        .refresh(refresh),
        .addr(addr), 
        .din(din), 
        .dout(dout), 
        .dout32(dout32), 
        .data_ready(data_ready),
        .busy(busy), 

        .SDRAM_DQ(SDRAM_DQ_o), 
        .SDRAM_DQ_o(SDRAM_DQ_i), 
        .SDRAM_A(SDRAM_A), 
        .SDRAM_BA(SDRAM_BA), 
        .SDRAM_nCS(SDRAM_nCS), 
        .SDRAM_nWE(SDRAM_nWE), 
        .SDRAM_nRAS(SDRAM_nRAS),
        .SDRAM_nCAS(SDRAM_nCAS), 
        .SDRAM_CLK(SDRAM_CLK), 
        .SDRAM_CKE(SDRAM_CKE),
        .SDRAM_DQM(SDRAM_DQM)
    );

    // Clock generation
    always begin
        #5 clk = ~clk;
    end

    // Display statements
    always @(rd, resetn, wr, addr, din, dout, data_ready, busy) begin
        $display("Time: %0t RST#: %b RD: %b WR: %b ADDR: %0d DIN: %h DOUT: %h DOUT32:%h RDY: %b BUSY: %b", 
            $time, resetn, rd, wr, addr, din, dout, dout32, data_ready, busy);
    end

    //always @(SDRAM_DQ_o, SDRAM_DQ_i, SDRAM_A, SDRAM_BA, SDRAM_nCS, SDRAM_nWE, SDRAM_nRAS, SDRAM_nCAS, SDRAM_DQM) begin
    //    $display("Time: %0t SD_DQ_o: %h SD_DQ_i: %h SD_A: %0d SD_BA: %b SD_nCS: %b SD_CMD: %d SD_nWE: %b SD_nRAS: %b SD_nCAS: %b SD_DQM: %b", 
    //            $time, SDRAM_DQ_o, SDRAM_DQ_i, SDRAM_A, SDRAM_BA, SDRAM_nCS, SDRAM_CMD, SDRAM_nWE, SDRAM_nRAS, SDRAM_nCAS, SDRAM_DQM);
    //end
    
    // Test sequence
    initial begin
        // Initial conditions
        clk = 1;
        resetn = 0;

        rd = 0;
        wr = 0;
        addr = 0;
        din = 0;
        refresh = 0;

        #10;
        resetn = 1;

        #300000;
   
        // Start simple tests
        #1000;
        $display("Time: %0t | WRITE ADDRESS 1 ----------------------------------------------", $time);
        wr = 1;
        addr = 23'd1;
        din = 8'hA5;

        #20;
        wr = 0;
        addr = 0;
        din = 0;
 
        //#50;
        #60;
        $display("Time: %0t | READ ADDRESS 1 ----------------------------------------------", $time);
        rd = 1;
        addr = 23'd1;

        #10;
        rd = 0;

        // ... Continue adding test sequences ...

        #100;
        $finish;
    end

endmodule
