`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/30/2021 02:08:34 PM
// Design Name: 
// Module Name: Completion Queue
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

module CompletionQueue( // Store does not broadcast to the CDB
    input CLK,
    input RS_tag_type TAG_IN1, // Inputs from LOAD_1
    input [31:0] DATA_IN1,
    input RS_tag_type TAG_IN2, // Inputs from LOAD_2
    input [31:0] DATA_IN2,
    input RS_tag_type TAG_IN3, // Inputs from ALU_1
    input [31:0] DATA_IN3,
    input RS_tag_type TAG_IN4, // Inputs from ALU_2
    input [31:0] DATA_IN4,
    
    output cdb_t CDB_OUT
    );
    
    cdb_t completion_queue [$:15];
    cdb_t cdb_out; // intermediate
    cdb_t in1, in2, in3, in4;
    
    always_comb
    begin
        in1.tag = TAG_IN1;
        in1.data = DATA_IN1;
    end
    
    always_comb
    begin
        in2.tag = TAG_IN2;
        in2.data = DATA_IN2;
    end
    
    always_comb
    begin
        in3.tag = TAG_IN3;
        in3.data = DATA_IN3;
    end
    
    always_comb
    begin
        in4.tag = TAG_IN4;
        in4.data = DATA_IN4;
    end
    
    always_ff @(posedge CLK) // add to completion queue
    begin
        if (TAG_IN1 != INVALID) completion_queue.push_back(in1);
        if (TAG_IN2 != INVALID) completion_queue.push_back(in2);
        if (TAG_IN3 != INVALID) completion_queue.push_back(in3);
        if (TAG_IN4 != INVALID) completion_queue.push_back(in4);
    end
    
    always_ff @(posedge CLK) // broadcast to CDB
    begin
        if (completion_queue.size() != 0) cdb_out = completion_queue.pop_front();  
    end
    
    assign CDB_OUT = cdb_out;
    
endmodule
