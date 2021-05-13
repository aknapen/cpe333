`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/11/2021 10:42:38 AM
// Design Name: 
// Module Name: InstructionFetch
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module InstructionFetch(
    input CLK,
    input EXT_RESET,

    input [31:0] jalr_pc, branch_pc, jump_pc,
    input [2:0] pc_source,
    input ld_haz,

    output [31:0] fetch_pc,
    output memRead1
    );
    
    logic [31:0]  next_pc, pc_in, pc_out;   
    logic pcWrite;

    assign pcWrite = ~ld_haz; // only allow new PC through if we're not stalling
    
    // Creates a 4-to-1 multiplexor used to select the source of the next PC
    Mult4to1 PCdatasrc (next_pc, jalr_pc, branch_pc, jump_pc, pc_source, pc_in);

    //PC is byte-addressed but our memory is word addressed 
    ProgCount PC (.PC_CLK(CLK), .PC_RST(EXT_RESET), .PC_LD(pcWrite),
                  .PC_DIN(pc_in), .PC_COUNT(pc_out));  
                   
    // Need to re-fetch decode instruction on a load-use hazard
    always_comb
    begin
        if (~ld_haz)              
            next_pc = pc_out + 4;    //PC is byte aligned, memory is word aligned (NEED TO CALC next_pc HERE SO IT'S READY BY NEXT INSTRUCTION 
        else
            next_pc = pc_out;                         
    end    
    
    assign memRead1 = ~ld_haz; // Want to read contingent on no load hazards
    assign fetch_pc = ld_haz ? pc_out -4 : pc_out;
    
//    always_comb
//    begin
//        fetch_pc = pc_out;
//        if (ld_haz) fetch_pc = pc_out-4;
//    end
    
endmodule
