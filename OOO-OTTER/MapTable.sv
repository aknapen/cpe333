`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  Christian Rapp
// 
// Create Date: 
// Design Name: 
// Module Name: ChristianRapp
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
    

module MapTable(
    input CLK,
    input opcode_t task_opcode,
    input rs1_used, rs2_used,
    input [4:0] rs1_addr, rs2_addr, rd_addr, // assign issue_tag to rd_addr in map table, possibly replace rs1/rs2 vals with tags
//    input RS_tag_type CDB_tag, // tag broadcast on the CDB
//    input [31:0] CDB_val, // value broadcast on the CDB
    input cdb_t cdb_in, // tag-value pair broadcast on the CDB
    input RS_tag_type issue_tag, // RS of issued task
    
    output RS_tag_type T1, T2, T3, // tags for the RS, T3 holds rs2 data tag for stores
    output [31:0] reg_data, // data to write to reg file
    output [4:0] writeReg_addr, // register # to write to in reg file
    output reg_valid // enable for writing to reg file
);
        
    MTEntry_type map_table [31:0];
    logic [31:0] reg_data_inter ;
    logic [4:0] writeReg_addr_inter;
    logic reg_valid_inter;
    
    initial begin
        for (int i =0; i<32; i++)
        begin
            map_table[i].tag = INVALID;
            map_table[i].busy = 0;
        end
        
        
    end
    
    always_ff @(posedge CLK) // map destination register of issued task in map table
    begin
        map_table[rd_addr].tag = issue_tag;
        map_table[rd_addr].busy = 1;
    end
    
    always_comb // send V1,V2 tags if needed
    begin
        T1 = INVALID; T2 = INVALID; T3 = INVALID;
        if (map_table[rs1_addr].busy && rs1_used) T1 = map_table[rs1_addr].tag;
        if (map_table[rs2_addr].busy &&  task_opcode != STORE && task_opcode != LOAD) T2 = map_table[rs2_addr].tag; // T2 will always be invalid for a STORE since we used immed as operand
        if (map_table[rs2_addr].busy && task_opcode == STORE) T3 = map_table[rs2_addr].tag;
    end
    
    // Comparator to send data to reg file
    always_ff @(posedge CLK)
    begin
        reg_valid_inter = 0;
        for (int i =0; i < 32; i++)
        begin
            if (map_table[i].tag == cdb_in.tag) 
            begin
                reg_data_inter = cdb_in.data;
                reg_valid_inter = 1;
                writeReg_addr_inter = i;
                map_table[i].busy = 0; // free up register after it's been written to
                map_table[i].tag = INVALID;
            end    
        end    
    end 
    
    assign reg_data = reg_data_inter;
    assign reg_valid = reg_valid_inter;
    assign writeReg_addr = writeReg_addr_inter;
    
endmodule

