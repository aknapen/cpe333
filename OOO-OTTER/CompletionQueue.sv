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
    
    always_ff @(posedge CLK) // add to completion queue
    begin
        if (TAG_IN1 != INVALID) completion_queue.push_back({TAG_IN1, DATA_IN1});
        if (TAG_IN2 != INVALID) completion_queue.push_back({TAG_IN2, DATA_IN2});
        if (TAG_IN3 != INVALID) completion_queue.push_back({TAG_IN3, DATA_IN3});
        if (TAG_IN4 != INVALID) completion_queue.push_back({TAG_IN4, DATA_IN4});
    end
    
    always_ff @(posedge CLK) // broadcast to CDB
    begin
        if (completion_queue.size() != 0) cdb_out = completion_queue.pop_front();  
    end
    
    assign CDB_OUT = cdb_out;
    
endmodule
