// Simple SDRAM controller for Tang 20k
// nand2mario
// 
// 2023.7: add buffers to din, dout and addr for ease-of-use.
// 2023.3: ported to use GW2AR-18's embedded 64Mbit SDRAM.
//         changed to byte-based access.
// 2022.9: iniital version.
//
// This is a byte-based, low-latency and non-bursting controller for the embedded SDRAM
// on Tang Nano 20K. The SDRAM module is 64Mbit 32bit. (2K rows x 256 columns x 4 banks x 32 bits).
//
// Under default settings (max 66.7Mhz):
// - Data read latency is 4 cycles. 
// - Read/write operations take 5 cycles to complete. There's no overlap between
//   reads/writes.
// - All reads/writes are done with auto-precharge. So user does not need to deal with
//   row activations and precharges.
// - SDRAMs need periodic refreshes or they lose data. So they provide an "auto-refresh"
//   function to do one row of refresh. This "auto-refresh" operation is controlled with
//   the 'refresh' input. 4096 or more refreshes should happen in any 64ms for the memory
//   to not lose data. So the main circuit should invoke auto-refresh at least once 
//   **every ~15us**.
//
// Finally you need a 180-degree phase-shifted clock signal (clk_sdram) for SDRAM. 
// This can be generated with PLL's clkoutp output.
//

module sdram
#(
    // Clock frequency, 100Mhz with current set of T_xx/CAS parameters.
    parameter         FREQ = 100_000_000,  
    parameter         DATA_WIDTH = 32,
    parameter         ROW_WIDTH = 11,  // 2K rows
    parameter         COL_WIDTH = 8,   // 256 words per row (1Kbytes)
    parameter         BANK_WIDTH = 2,  // 4 banks

    parameter         AUTO_REFRESH_TIMER = 1,  // enable auto-refresh

    // Time delays for 100Mhz max clock CL3 (min clock cycle 15ns)
    // The SDRAM supports max 166.7Mhz (RP/RCD/RC need changes)
    parameter [4:0]   CAS  = 5'd3,     // 2/3 cycles, set in mode register
    parameter [4:0]   T_WR = 5'd2,     // 2 cycles, write recovery
    parameter [4:0]   T_MRD= 5'd2,     // 2 cycles, mode register set
    parameter [4:0]   T_RP = 5'd2,     // 20ns, precharge to active
    parameter [4:0]   T_RCD= 5'd2,     // 20ns, active to r/w
    parameter [4:0]   T_RC = 5'd6      // 60ns, ref/active to ref/active
)
(
    // SDRAM side interface
    input [DATA_WIDTH-1:0]      SDRAM_DQ,
    output [DATA_WIDTH-1:0]      SDRAM_DQ_o,
    output reg [ROW_WIDTH-1:0]  SDRAM_A,
    output reg [BANK_WIDTH-1:0] SDRAM_BA,
    output            SDRAM_nCS,    // not strictly necessary, always 0
    output reg        SDRAM_nWE,
    output reg        SDRAM_nRAS,
    output reg        SDRAM_nCAS,
    output            SDRAM_CLK,
    output            SDRAM_CKE,    // not strictly necessary, always 1
    output reg  [3:0] SDRAM_DQM,
    
    // Logic side interface
    input             clk,
    input             clk_sdram,    // phase shifted from clk (normally 180-degrees)
    input             resetn,
    input             rd,           // command: read
    input             wr,           // command: write
    input             refresh,      // command: auto refresh. 4096 refresh cycles in 64ms. Once per 15us.
    input      [22:0] addr,         // byte address, buffered at rd/wr pulse time
    input       [7:0] din,          // data input, buffered at wr pulse time
    output reg [7:0] dout,         // data output, available 4 cycles after rd becomes 1
                                    // output is buffered until next read request
    output reg [DATA_WIDTH-1:0] dout32, // 32-bit data output
    output reg        data_ready,   // available 6 cycles after wr is set
    output reg        busy          // 0: ready for next command
);

// Tri-state DQ input/output
reg dq_oen;         // 0 means output
reg [DATA_WIDTH-1:0] dq_out;
assign SDRAM_DQ_o = dq_out;
reg [DATA_WIDTH-1:0] dq_in;     // DQ input

always @(posedge clk_sdram) begin
    dq_in <= SDRAM_DQ;
end

reg [1:0] off;          // byte offset

assign SDRAM_CLK = clk_sdram;
assign SDRAM_CKE = 1'b1;
assign SDRAM_nCS = 1'b0;

reg [2:0] state;
localparam INIT = 3'd0;
localparam CONFIG = 3'd1;
localparam IDLE = 3'd2;
localparam READ = 3'd3;
localparam WRITE = 3'd4;
localparam REFRESH = 3'd5;
wire running = (state != INIT) && (state != CONFIG);

// RAS# CAS# WE#
localparam CMD_SetModeReg=3'b000;  //0
localparam CMD_AutoRefresh=3'b001; //1
localparam CMD_PreCharge=3'b010; //2
localparam CMD_BankActivate=3'b011; //3
localparam CMD_Write=3'b100; //4
localparam CMD_Read=3'b101; //5
localparam CMD_NOP=3'b111; //7

localparam [2:0] BURST_LEN = 3'b0;      // burst length 1
localparam BURST_MODE = 1'b0;           // sequential
localparam [10:0] MODE_REG = {4'b0, CAS[2:0], BURST_MODE, BURST_LEN};

reg cfg_now;            // pulse for configuration
reg [4:0] cycle;        // each operation (config/read/write) are max 7 cycles
reg [7:0] din_buf;      // set at wr=1 pulse time
reg [22:0] addr_buf;

reg refresh_needed;
reg refresh_executed;                           // pulse from main FSM

//
// SDRAM state machine
//
always @(posedge clk) begin
    data_ready <= 1'b0;
    refresh_executed <= 1'b0;
    cycle <= cycle == 5'd31 ? 5'd31 : cycle + 5'd1;
    // defaults
    {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_NOP; 
    casex ({state, cycle})
        // wait 200 us on power-on
        {INIT, 5'bxxxx} : if (cfg_now) begin
            state <= CONFIG;
            cycle <= 0;
        end

        // configuration sequence
        //  cycle  / 0 \___/ 1 \___/ 2 \___/ ... __/ 6 \___/ ...___/10 \___/11 \___/ 12\___
        //  cmd            |PC_All |Refresh|       |Refresh|       |  MRD  |       | _next_
        //                 '-T_RP--`----  T_RC  ---'----  T_RC  ---'------T_MRD----'
        {CONFIG, 5'd0} : begin
            // precharge all
            $display ("%m : at time %t CONFIG : precharge all", $time);
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_PreCharge;
            SDRAM_A[10] <= 1'b1;
        end
        {CONFIG, T_RP} : begin
            // 1st AutoRefresh
            $display ("%m : at time %t CONFIG : 1st AutoRefresh", $time);
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AutoRefresh;
        end
        {CONFIG, T_RP+T_RC} : begin
            // 2nd AutoRefresh
            $display ("%m : at time %t CONFIG : 2nd AutoRefresh", $time);
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AutoRefresh;
        end
        {CONFIG, T_RP+T_RC+T_RC} : begin
            // set register
            $display ("%m : at time %t CONFIG : set register", $time);
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_SetModeReg;
            SDRAM_A[10:0] <= MODE_REG;
        end
        {CONFIG, T_RP+T_RC+T_RC+T_MRD} : begin
            $display ("%m : at time %t CONFIG : Config done", $time);
            state <= IDLE;
            busy <= 1'b0;              // init&config is done
        end
        
        // read/write/refresh
        {IDLE, 5'bxxxx}: if (rd | wr) begin
            $display ("%m : at time %t IDLE : bank activate for READ/WRITE", $time);
            // bank activate
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_BankActivate;
            SDRAM_BA <= addr[ROW_WIDTH+COL_WIDTH+BANK_WIDTH-1+2 : ROW_WIDTH+COL_WIDTH+2];    // bank id
            SDRAM_A <= addr[ROW_WIDTH+COL_WIDTH-1+2:COL_WIDTH+2];      // 12-bit row address
            state <= rd ? READ : WRITE;
            addr_buf <= addr;
            if (wr) din_buf <= din;
            cycle <= 5'd1;
            busy <= 1'b1;
        end else if (refresh_needed || refresh) begin
            $display ("%m : at time %t IDLE : REFRESH initiated", $time);
            // auto-refresh
            // no need for precharge-all b/c all our r/w are done with auto-precharge.
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AutoRefresh;
            state <= REFRESH;
            cycle <= 5'd1;
            busy <= 1'b1;
            refresh_executed <= 1'b1;
        end

        // read sequence
        //  cycle  / 0 \___/ 1 \___/ 2 \___/ 3 \___/ 4 \___/ 5 \___
        //  rd     /       \_______________________________
        //  cmd            |Active | Read  |  NOP  |  NOP  | _Next_
        //  DQ                                     |  Dout |
        //  data_ready ____________________________/       \_______   
        //  busy   ________/                               \_______
        //                 `-T_RCD-'------CAS------'
        {READ, T_RCD}: begin
            $display ("%m : at time %t READ : set auto precharge/column address", $time);
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_Read;
            SDRAM_A[10] <= 1'b1;        // set auto precharge
            SDRAM_A[9:0] <= {1'b0, addr_buf[COL_WIDTH-1+2:2]};  // column address
            SDRAM_DQM <= 4'b0;
            off <= addr_buf[1:0];
        end
        {READ, T_RCD+CAS+5'd1}: begin
            $display ("%m : at time %t READ : data in %h", $time, dq_in);
            $display ("%m : at time %t READ : done", $time);
            dout <= off == 0 ? dq_in[7:0] :
                    off == 1 ? dq_in[15:8] :
                    off == 2 ? dq_in[23:16] : dq_in[31:24];
            dout32 <= dq_in;
            data_ready <= 1'b1;
            busy <= 0;
            state <= IDLE;
        end

        // write sequence
        //  cycle / 0 \___/ 1 \___/ 2 \___/ 3 \___/ 4 \___/ 5 \___
        //  wr    /       \_______________________________
        //  cmd           |Active | Write |  NOP  |  NOP  | _Next_
        //  DQ                    | Din   |
        //  busy   _______/                               \_______
        //                `-T_RCD-'-------T_WR+T_RP-------'
        {WRITE, T_RCD}: begin
            $display ("%m : at time %t WRITE : set auto precharge/column address", $time);
            {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_Write;
            SDRAM_A[10] <= 1'b1;        // set auto precharge
            SDRAM_A[9:0] <= {1'b0, addr_buf[COL_WIDTH-1+2:2]};  // column address
            SDRAM_DQM <= addr_buf[1:0] == 2'd0 ? 4'b1110 :
                         addr_buf[1:0] == 2'd1 ? 4'b1101 :
                         addr_buf[1:0] == 2'd2 ? 4'b1011 : 4'b0111;     // only write the correct byte
            off <= addr_buf[1:0];
            dq_out <= {din_buf,din_buf,din_buf,din_buf};
            dq_oen <= 1'b0;                 // DQ output on
        end
        {WRITE, T_RCD+5'd1}: begin
            $display ("%m : at time %t WRITE : output enable", $time);
            dq_oen <= 1'b1;
        end
        {WRITE, T_RCD+T_WR+T_RP+5'd1}: begin  // 2+2+1
            $display ("%m : at time %t WRITE : done", $time);
            busy <= 0;
            state <= IDLE;
        end

        // refresh sequence
        //  cycle   / 0 \___/ 1 \___/ 2 \___/ 3 \___/ 4 \___/ 5 \___
        //  refresh /       \_______________________________
        //  cmd             |Refresh|  NOP  |  NOP  |  NOP  | _Next_
        //  busy     _______/                               \_______
        //                  `------------- T_RC ------------'
        {REFRESH, T_RC}: begin
            $display ("%m : at time %t REFRESH : done", $time);
            state <= IDLE;
            busy <= 0;
        end
    endcase

    if (~resetn) begin
        busy <= 1'b1;
        dq_oen <= 1'b1;         // turn off DQ output
        SDRAM_DQM <= 4'b0;
        state <= INIT;
        dout <= 8'd0;
        dout32 <= 8'd0;
    end
end


//
// Generate cfg_now pulse after initialization delay (normally 200us)
//
reg  [14:0]   rst_cnt;
reg rst_done, rst_done_p1, cfg_busy;
  
always @(posedge clk) begin
    rst_done_p1 <= rst_done;
    cfg_now     <= rst_done & ~rst_done_p1;// Rising Edge Detect

    if (rst_cnt != FREQ / 1000 * 200 / 1000) begin      // count to 200 us
        rst_cnt  <= rst_cnt[14:0] + 1;
        rst_done <= 1'b0;
        cfg_busy <= 1'b1;
    end else begin
        rst_done <= 1'b1;
        cfg_busy <= 1'b0;
    end

    if (~resetn) begin
        rst_cnt  <= 15'd0;
        rst_done <= 1'b0;
        cfg_busy <= 1'b1;
    end
end

localparam REFRESH_COUNT=FREQ/1000/1000*15;     // 15us refresh
reg [11:0] refresh_time;
always @(posedge clk) begin
    if (~resetn) begin
        refresh_time <= 0;
        refresh_needed <= 0;
    end else if (AUTO_REFRESH_TIMER & running) begin
        refresh_time <= (refresh_time == (REFRESH_COUNT*2-2)) ? (REFRESH_COUNT*2-2) : refresh_time + 1;
        if (refresh_time == REFRESH_COUNT) begin
            refresh_needed <= 1;
            $display ("%m : at time %t REFRESH NEEDED", $time);
        end
        if (refresh_executed) begin
            refresh_time <= refresh_time - REFRESH_COUNT;
            refresh_needed <= 0;
        end
    end
end


endmodule