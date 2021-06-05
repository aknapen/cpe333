`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: J. Callenes 
// 
// Create Date: 06/07/2018 05:03:50 PM
// Design Name: 
// Module Name: ArithLogicUnit
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

import cpu_types::*;

module OTTER_ALU(
    input [31:0] V1, V2,
    input V1_valid, V2_valid,
    input [3:0] alu_fun,
    input RS_tag_type rd_tag,
    
    output [31:0] CDB_val,
    output RS_tag_type CDB_tag,
    output done
);       
    // intermediates
    logic [31:0] ALUOut, cdb_val;
    RS_tag_type cdb_tag;
    logic complete;
  
//    initial begin complete = 1; end
    
    always_comb
    begin //reevaluate If these change
//        complete = 1;
        if (V1_valid && V2_valid)
        begin
            case(alu_fun)
                0:  ALUOut = V1 + V2 ;     //add
                8:  ALUOut = V1 - V2 ;     //su V2 
                6:  ALUOut = V1 | V2 ;     //or
                7:  ALUOut = V1 & V2 ;     //and
                4:  ALUOut = V1 ^ V2 ;     //xor
                5:  ALUOut =  V1 >>  V2 [4:0];    //srl
                1:  ALUOut =  V1 <<  V2 [4:0];    //sll
               13:  ALUOut =  $signed(V1) >>>  V2 [4:0];    //sra
                2:  ALUOut = $signed(V1) < $signed( V2 ) ? 1: 0;       //slt
                3:  ALUOut = V1 < V2 ? 1: 0;      //sltu
                9:  ALUOut = V1; //copy op1 (lui)
                10: ALUOut = V1 *  V2 ;
                default: ALUOut = 0; 
            endcase
        end
    end
    
    // Broadcast result on the CDB
    always_comb
    begin
        cdb_tag = INVALID; // don't want to add to commit queue until ready
        complete = 0;
        if (V1_valid && V2_valid)
        begin
            cdb_val = ALUOut;
            cdb_tag = rd_tag;
            complete = 1;
        end
    end
    
    assign CDB_val = cdb_val;
    assign CDB_tag = cdb_tag;
    assign done = complete;
    
endmodule
   
  