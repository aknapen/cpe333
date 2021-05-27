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
    input [1:0] pc_source,
    input ld_haz,

    output [31:0] fetch_pc_0, fetch_pc_1,
    output memRead1
    );
    
    logic [31:0]  next_pc_0, next_pc_1, pc_in_0, pc_in_1, pc_out_0, pc_out_1;   
    logic pcWrite;

//    assign pcWrite = ~ld_haz; // only allow new PC through if we're not stalling

    assign pcWrite = 1; // ignore hazards for now    
    
    // Creates a 4-to-1 multiplexor used to select the source of the next PC
    // For now, we will only use next_pc and ignore branches/jups
//    Mult4to1 PCdatasrc (next_pc, jalr_pc, branch_pc, jump_pc, pc_source,pc_in);
    MultPC PCdatasrc (.*);
    
    //PC is byte-addressed but our memory is word addressed 
    ProgCount PC (.PC_CLK(CLK), .PC_RST(EXT_RESET), .PC_LD(pcWrite),
                  .PC_DIN0(pc_in_0), .PC_DIN1(pc_in_1), .PC_COUNT0(pc_out_0), .PC_COUNT1(pc_out_1));  
                   
    // ignoring load-use hazards
    always_comb
    begin
        next_pc_0 = pc_out_0 + 8;    //PC is byte aligned, memory is word aligned (NEED TO CALC next_pc HERE SO IT'S READY BY NEXT INSTRUCTION 
        next_pc_1 = pc_out_1 + 8;                         
    end    
    
    assign memRead1 = 1;
    assign fetch_pc_0 = pc_out_0;
    assign fetch_pc_1 = pc_out_1;    
endmodule
