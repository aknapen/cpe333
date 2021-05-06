`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Keefe Johnson
//           Joseph Callenes
//           
// 
// Create Date: 02/06/2020 06:40:37 PM
// Design Name: 
// Module Name: dcache
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - 
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

package cache_def;

parameter int TAG_MSB = 31;
parameter int TAG_LSB = 12;

typedef struct packed{
    logic valid;
    logic dirty;
    logic [TAG_MSB:TAG_LSB] tag;
}cache_tag_type;

typedef struct {
    logic [9:0] index;
    logic we;
}cache_req_type;

//128-bit cache line
typedef logic [127:0] cache_data_type;

//CPU request (CPU ->cache controller)
typedef struct{
    logic [31:0] addr;
    logic [31:0] data;
    logic rw;
    logic valid;
}cpu_req_type;

//Cache result (cache controller -> CPU)
typedef struct {
    logic [31:0]data;
    logic ready;
}cpu_result_type;

//memory request (cache controller -> memory)
typedef struct {
    logic [31:0]addr;
    logic [127:0]data;
    logic rw;
    logic valid;
}mem_req_type;

//memory controller response (memory -> cache controller)
typedef struct {
cache_data_type data;
logic ready;
}mem_data_type;

endpackage

import cache_def::*;
import memory_bus_sizes::*; 

module L1_cache_data ( 
    input clk,
    input cache_req_type data_req,
    input cache_data_type data_write,
    input [3:0] be,
    input [1:0] block_offset,
    input from_ram,
    output cache_data_type data_read);
    
    cache_data_type data_mem[0:255];
    
    initial begin
        for(int i=0; i<256; i++)
            data_mem[i]='0;
    end

    always_ff @(posedge clk) begin
        if(data_req.we) begin
            if(from_ram) 
                data_mem[data_req.index] <= data_write;
            if(!from_ram) begin
              for (int b = 0; b < WORD_SIZE; b++) begin
                if (be[b]) begin
                    data_mem[data_req.index][block_offset*WORD_WIDTH+b*8+:8] <= data_write[block_offset*WORD_WIDTH+b*8+:8];  //[b*8+:8];
                end
              end
            end
        end
            
        data_read <= data_mem[data_req.index];
    end
endmodule

module L1_cache_tag (
    input logic clk,
    input cache_req_type tag_req,
    input cache_tag_type tag_write,
    output cache_tag_type tag_read);
    
   cache_tag_type tags [1023:0];
   
   initial begin
        for(int i=0; i<1023; i++)
            tags[i]='0;
    end
    
    always_comb // Asynchronous read
    begin   
        tag_read = tags[tag_req.index];
    end
    
    always_ff @(posedge clk) // Synchronous write
    begin
        if(tag_req.we)
        begin
            tags[tag_req.index].tag <= tag_write.tag;
            tags[tag_req.index].valid <= 1;
            tags[tag_req.index].dirty <= tag_write.dirty;
        end
        
    end
    
    
	// tag storage  
	
	// If tag_reg.we then tags[tag_req.index] <= tag_write
	// tag_read = tags[tag_req.index]
	// incluces valid and dirty bits
	// async read, sync write
	
	
endmodule


module dcache(
    input clk, RESET,
    i_mhub_to_dcache.device mhub,
    i_dcache_to_ram.controller ram
    );
    
    // Inputs to the cache
    // Will use mhub and ram output signals
    cpu_req_type cpu_req;     //CPU->cache
    mem_data_type mem_data;   //memory->cache
    
    // Outputs from the cache
    // Will set mhub and ram input signals
    mem_req_type mem_req;    //cache->memory
    cpu_result_type cpu_res;  //cache->CPU
    
    logic [1:0] block_offset;
    logic [3:0] be;
    logic from_ram;
    logic wait_read, next_wait_read;   
    
    
    typedef enum {idle, compare_tag, allocate, writeback} cache_state_type;
   
    cache_state_type state, next_state;

    cache_tag_type tag_read; // tag to be read from the cache tag module
    cache_tag_type tag_write; // for writing a tag to the cache tag module
    cache_req_type tag_req; // request info for reading from or writing to the cache tag module
    
    cache_data_type data_read; // data to be read form the cache data module
    cache_data_type data_write; // data to be written to the cache data module
    cache_req_type data_req; // request info for reading from or writing to the cache data module
    
    cpu_result_type next_cpu_res;
    
    // Get the necessary data from the mhub (mhub => cpu_req)
    assign cpu_req.addr = (mhub.write_addr_valid) ? mhub.write_addr : mhub.read_addr; // MUX for whether mhub is writing to or reading from cache
    assign cpu_req.data = mhub.write_data; // Take whatever write data the mhub has and check if it's valid later
    assign cpu_req.rw = (mhub.write_addr_valid) ? 1 : 0; // 1 if a write, 0 if a read
    assign cpu_req.valid = mhub.read_addr_valid || mhub.write_addr_valid; // don't know how to set this
	   
    // Send the necessary data to the mhub (cpu_res => mhub)
    assign mhub.read_addr_ready = cpu_res.ready;
    assign mhub.read_data = cpu_res.data;
    assign mhub.read_data_valid = 0; // don't know how to set this
    assign mhub.write_addr_ready = cpu_res.ready;
    assign mhub.write_resp_valid = 0; // don't know how to set this
    
    // Send the necessary data to the RAM (mem_req => ram)
    assign ram.read_addr = mem_req.addr;
    assign ram.read_addr_valid =  mem_req.valid && !mem_req.rw;
    assign ram.write_addr = mem_req.addr;
    assign ram.write_addr_valid = mem_req.valid && mem_req.rw; 
    assign ram.write_data = mem_req.data;
    
    // Get the necessary data from the RAM (ram => mem_data)
    assign mem_data.data = ram.read_data;
    assign mem_data.ready = (ram.read_addr_ready || ram.write_addr_ready);
    
    
	
	//FSM for Cache Controller
    
    always_ff @(posedge clk) begin
        state <= next_state;
    end
    
    always_comb 
    begin
        case(state)
            idle:
            begin
            
            end
            compare_tag:
            begin
            
            end
            
            allocate:
            begin
            
            end
            
            writeback:
            begin
            
            end
                    
        endcase
    end
    
	
	
	

    L1_cache_tag L1_tags(.*);
    L1_cache_data L1_data(.*);

endmodule
