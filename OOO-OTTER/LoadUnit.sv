`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/28/2021 01:38:53 PM
// Design Name: 
// Module Name: LoadUnit
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


module LoadUnit(
    input logic [31:0] V1, V2,
    input logic V1_valid, V2_valid,
    input RS_tag_type rd_tag,
    input logic [2:0] mem_type, 
    input logic mem_resp,
    input logic mem_resp_valid,
    input logic [31:0] mem_data_in,
    
    output logic [31:0] CDB_val,
    output RS_tag_type CDB_tag,
    output logic done,
    output logic [31:0] MEM_ADDR2,
    output logic MEM_SIGN,
    output logic [1:0] MEM_SIZE,
    output logic MEM_READ
    );
    
    // intermediates
    logic [31:0] cdb_val;
    RS_tag_type cdb_tag;
    logic complete;
    logic [31:0] mem_addr;
    logic mem_read;
    
    initial 
    begin 
        complete = 1; 
        mem_read = 0;
    end
    
    always_comb
    begin
        if (mem_resp && mem_resp_valid)  // Broadcast result over CDB
        begin
            complete = 0;
            if (V1_valid && V2_valid)
            begin
                cdb_val = mem_data_in;
                cdb_tag = rd_tag;
                complete = 1;
            end
        end
    end
    
    always_comb
    begin
        if (V1_valid && V2_valid)
        begin
            mem_read = 1;
            mem_addr = V1 + V2; // calculate address to load from
        end
    end
    
    assign CDB_val = cdb_val;
    assign CDB_tag = cdb_tag;
    assign done = complete;
    assign MEM_ADDR2 = mem_addr;
    assign MEM_SIGN =  mem_type[2];
    assign MEM_SIZE = mem_type[1:0];
    assign MEM_READ = mem_read;
    
endmodule
