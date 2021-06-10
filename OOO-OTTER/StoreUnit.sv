`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/28/2021 01:37:29 PM
// Design Name: 
// Module Name: StoreUnit
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


module StoreUnit(
    input logic [31:0] V1, V2, V3,
    input logic V1_valid, V2_valid, V3_valid,
    input RS_tag_type rd_tag,
    input logic [2:0] mem_type,    
    input mem_resp,
    input mem_resp_valid,
    
    output logic done,
    output logic [31:0] MEM_ADDR2,
    output logic MEM_WRITE,
    output logic [31:0] MEM_WRITE_DATA,
    output logic MEM_SIGN,
    output logic [1:0] MEM_SIZE
    );
    
    // intermediates
    logic [31:0] mem_addr;
    logic mem_write, complete;
    
//    initial begin complete = 1; end
    
    always_comb
    begin
        complete = 1;
        mem_write = 0;
        mem_addr = V1 + V2; // calculate address into memory to store
        if (V1_valid && V2_valid && V3_valid)
        begin
            complete = 0;
            mem_write = 1;
        end
        
        if (mem_resp_valid) complete = mem_resp;
        
    end
    
//    always_comb
//    begin
//        if (mem_resp_valid) complete = mem_resp;
//    end
  
    assign done = complete;
    assign MEM_ADDR2 = mem_addr;
    assign MEM_WRITE = mem_write;
    assign MEM_WRITE_DATA = V3;
    assign MEM_SIGN = mem_type[2];
    assign MEM_SIZE = mem_type[1:0];
endmodule
