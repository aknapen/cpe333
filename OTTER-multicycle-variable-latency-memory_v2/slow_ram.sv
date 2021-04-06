`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Keefe Johnson
// 
// Create Date: 02/06/2020 06:40:37 PM
// Updated Date: 02/13/2020 08:00:00 AM
// Design Name: 
// Module Name: slow_ram
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - Updated for use without caches and AXI bus connections
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


import memory_bus_sizes::*;

module slow_ram #(
    parameter MEM_DELAY = 10,  // accept/process one command every MEM_DELAY+1 clock cycles
    parameter RAM_DEPTH = -1,  // define in parent
    parameter INIT_FILENAME = ""  // define in parent
    )(
    input clk,
    axi_bus_ro.device mhub1,
    axi_bus_rw.device mhub2
    );

    logic [ADDR_WIDTH-1:0] rami_addr, ramd_addr;
    logic [DATA_WIDTH-1:0] rami_din, rami_dout, ramd_din, ramd_dout;
    logic rami_en, ramd_en, ramd_we; 
        
   /* delay_access #(
        .PORT_NAME("Instruction"), .MEM_DELAY(0)
    ) delay_rami (
        .clk(clk), .mhub(mhub1), //.baddr(mhub1.read_addr), .en(icache.en), .we('0), .din('0), .dout(mhub1.read_data), .hold(mhub1.read_addr_ready),
        .ram_addr(rami_addr), .ram_en(rami_en), .ram_we(), .ram_din(), .ram_dout(rami_dout)
    );*/
    delay_access #(
        .PORT_NAME("Data"), .MEM_DELAY(MEM_DELAY)
    ) delay_ramd (
        .clk(clk), .mhub(mhub2), //.baddr(dcache.baddr), .en(dcache.en), .we(dcache.we), .din(dcache.din), .dout(dcache.dout), .hold(dcache.hold),
        .ram_addr(ramd_addr), .ram_en(ramd_en), .ram_we(ramd_we), .ram_din(ramd_din), .ram_dout(ramd_dout)
    );
    xilinx_bram_tdp_nc_nr #(
        .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH), .RAM_DEPTH(RAM_DEPTH),
        .INIT_FILENAME(INIT_FILENAME)           //BLOCK_WIDTH
    ) bram (
        //.clka(clk), .addra(rami_addr), .dina('0), .douta(rami_dout), .ena(rami_en), .wea('0),
        .clka(clk), .addra(mhub1.read_addr[ADDR_WIDTH-1:2]), .dina('0), .douta(mhub1.read_data), .ena(mhub1.read_addr_valid), .wea('0), 
        .clkb(clk), .addrb(ramd_addr[ADDR_WIDTH-1:2]), .dinb(ramd_din), .doutb(ramd_dout), .enb(ramd_en), .web(ramd_we) 
    );

    
endmodule

module delay_access #(
    PORT_NAME = "",  // define in parent, for debug output labeling
    MEM_DELAY = -1  // define in parent
    )(
    input clk,
    axi_bus_rw.device mhub,
    //input [ADDR_WIDTH-1:0] addr,
    //input en, we,
    //input [DATA_WIDTH-1:0] din,
    
    //output [DATA_WIDTH-1:0] dout,
    output [ADDR_WIDTH-1:0] ram_addr,
    output ram_en, ram_we,
    output [DATA_WIDTH-1:0] ram_din,
    input [DATA_WIDTH-1:0] ram_dout
    
    );

    localparam CNT_WIDTH = (MEM_DELAY == 0) ? 1 : $clog2(MEM_DELAY+1);
    logic [CNT_WIDTH-1:0] r_cycle_cnt = 0;
    logic [ADDR_WIDTH-1:0] r_addr;
    logic [DATA_WIDTH-1:0] r_write_data;
    logic r_busy=0;
    logic r_ram_en, r_ram_we;

    assign mhub.read_data_valid = (r_cycle_cnt ==MEM_DELAY);
    assign mhub.read_addr_ready = !r_busy;
    assign mhub.write_addr_ready = !r_busy;
     
    assign ram_we = r_ram_we; 
    assign ram_en = r_ram_en;
    assign ram_we = r_ram_we;
    assign ram_addr = r_addr;
    assign ram_din = r_write_data;
    assign mhub.read_data = ram_dout;

    always_ff @(posedge clk) begin
        r_ram_en <= 0;
        if(r_busy) begin
            if(r_cycle_cnt == MEM_DELAY) begin
                r_cycle_cnt <=0;
                r_busy <=0;
                r_ram_we <=0;
                r_addr <=0;
                r_write_data <=0;
            end else  begin
                r_cycle_cnt <= r_cycle_cnt + 1;
            end
            if(r_cycle_cnt == MEM_DELAY-1) begin
                r_ram_en <=1;
            end
        end
  
        if(MEM_DELAY>0 && !r_busy) begin               //accept and save memory request (addr, we, write_data if write)
            if(mhub.read_addr_valid) begin
                r_addr <= mhub.read_addr;
                r_ram_we <= 0;
                r_busy <=1;
            end
            if(mhub.write_addr_valid) begin
                r_addr <= mhub.write_addr;
                r_write_data <= mhub.write_data;
                r_ram_we <= 1;
                r_busy <=1;
            end
        end     
    end
    
endmodule




/*
    logic s_ready;  // if command present, tells controller to hold it's command steady while we process
    logic s_available;  // available unless processing, but also includes all of the acceptance partial cycle
    logic s_processing;  // includes acceptance partial cycle
    logic s_accepting, s_in_delay_cycles;  // if s_processing, exactly one true, otherwise all false
    logic s_reading, s_writing;  // if s_processing, exactly one true, otherwise all false
    logic s_in_final_cycle;  // the operation completes on the edge after this cycle, which may be the acceptance cycle if MEM_DELAY==0
    logic s_we;  // command or saved
    
    logic [ADDR_WIDTH-1:0] s_addr;  // command or saved
    logic [DATA_WIDTH-1:0] s_write_data;  // command or saved

    assign s_in_delay_cycles = r_cycle_cnt > 0;
    assign s_available = !s_in_delay_cycles;
    assign s_accepting = s_available && en;
    assign s_processing = s_accepting || s_in_delay_cycles;
    assign s_in_final_cycle = s_processing && r_cycle_cnt == MEM_DELAY;
    assign s_we = s_accepting ? we : r_we;
    assign s_reading = s_processing && !s_we;
    assign s_writing = s_processing && s_we;
    assign s_addr = s_accepting ? addr : r_addr;
    assign s_din = s_accepting ? din : r_din;
    assign s_hold = en ? ((s_reading && !s_in_final_cycle) || (s_writing && !s_accepting)) : 0;

    assign hold = s_hold;
    assign dout = ram_dout;

    assign ram_baddr = s_baddr;
    assign ram_en = s_in_final_cycle;
    assign ram_we = s_writing;
    assign ram_din = s_din;

    always_ff @(posedge clk) begin
        if (s_in_final_cycle) begin
            r_cycle_cnt <= 0;
        end else if (s_processing) begin
            r_cycle_cnt <= r_cycle_cnt + 1;
        end
        if (s_accepting && !s_in_final_cycle) begin
            r_baddr <= baddr;
            r_we <= we;
            if (s_writing) begin
                r_din <= din;
            end
        end
    end
  */  