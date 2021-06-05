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
  
    initial begin complete = 1; end
    
    always_comb
    begin //reevaluate If these change
        if (V1_valid && V2_valid)
        begin
            case(alu_fun)
                0:  ALUOut = A + B;     //add
                8:  ALUOut = A - B;     //sub
                6:  ALUOut = A | B;     //or
                7:  ALUOut = A & B;     //and
                4:  ALUOut = A ^ B;     //xor
                5:  ALUOut =  A >> B[4:0];    //srl
                1:  ALUOut =  A << B[4:0];    //sll
               13:  ALUOut =  $signed(A) >>> B[4:0];    //sra
                2:  ALUOut = $signed(A) < $signed(B) ? 1: 0;       //slt
                3:  ALUOut = A < B ? 1: 0;      //sltu
                9:  ALUOut = A; //copy op1 (lui)
                10: ALUOut = A * B;
                default: ALUOut = 0; 
            endcase
        end
    end
    
    // Broadcast result on the CDB
    always_comb
    begin
        cdb_tag = INVALID; // don't want to add to commit queue until ready
        complete = 0;
        if (!CDB_busy && V1_valid && V2_valid)
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
   
  