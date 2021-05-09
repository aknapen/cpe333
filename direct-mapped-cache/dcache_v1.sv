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
    
    always_comb // asynchronous read
    begin
        data_read <= data_mem[data_req.index];
    end
    
    always_ff @(posedge clk) begin // Synchronous write
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
            
    end
endmodule

module L1_cache_tag (
    input logic clk,
    input cache_req_type tag_req,
    input cache_tag_type tag_write,
    output cache_tag_type tag_read);
    
   cache_tag_type tags [0:255];
   
   initial begin
        for(int i=0; i<256; i++)
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
endmodule


module dcache(
    input clk, RESET,
    axi_bus_rw.device mhub,
    axi_bus_rw.controller ram
    );
		
    // Inputs/Outputs from the cache tag module
    cache_tag_type tag_read; // tag to be read from the cache tag module
    cache_tag_type tag_write; // for writing a tag to the cache tag module
    cache_req_type tag_req; // request info for reading from or writing to the cache tag module
    
    // Inputs/Outputs from the cache data module
    logic [1:0] block_offset; // block_offset tells you which word to read from a block
    logic [3:0] be; // be tells you which byte to write to in a block
    logic from_ram; // tells you if the data you are writing to the cache is from the RAM or not
    logic wait_read, next_wait_read; 
    cache_data_type data_read; // data to be read form the cache data module
    cache_data_type data_write; // data to be written to the cache data module
    cache_req_type data_req; // request info for reading from or writing to the cache data module
	
	typedef enum {idle, compare_tag, allocate, writeback} cache_state_type;
    cache_state_type state, next_state;
    
	//FSM for Cache Controller
	
	assign write_hit = ((mhub.write_addr[TAG_MSB:TAG_LSB] == tag_read.tag) && mhub.write_addr_valid) && tag_read.valid;
	assign read_hit = ((mhub.read_addr[TAG_MSB:TAG_LSB] == tag_read.tag) && mhub.read_addr_valid) && tag_read.valid;
    
    always_ff @(posedge clk) begin
        state <= next_state;
    end
    
    always_comb 
    begin
        case(state)
            compare_tag:
            begin
                if (mhub.read_addr_valid) // if we are trying to read or write
                begin
                    tag_req.we = 0;
                    tag_req.index = mhub.read_addr[11:4];
                    data_req.we = 0;
                    data_req.index = mhub.read_addr[11:4];
                end
                
                else if (mhub.write_addr_valid) // if we're trying to write
                begin
                    tag_req.we = 0;
                    data_req.we = 1;
                    data_write = mhub.write_data << (block_offset*WORD_WIDTH);
                end
                
                mhub.read_addr_ready = 1;
                mhub.write_addr_ready = 1;
                mhub.read_data_valid = 0;
                mhub.write_resp_valid = 0;
                
                // Next Stage calculation
                if (write_hit) // successful write
                begin
                    tag_req.we = 1;
                    tag_write.dirty = 1; // data in cache no longer corresponds to data in memory
                    mhub.write_resp_valid = 1;
                    from_ram = 0; // writing from CPU data, not from RAM
                    
                    next_state = compare_tag; // stay in compare_tag on a successful write to cache
                end
                
                else if (read_hit) // successful read
                begin
                    mhub.read_data = data_read >> (block_offset*WORD_WIDTH);
                    mhub.read_data_valid = 1;
                    next_state = compare_tag; // stay in compare_tag on a successful read from cache
                end
                
                else if (tag_read.dirty && !(read_hit || write_hit) && (mhub.read_addr_valid || mhub.write_addr_valid)) 
                begin
                    ram.write_data = data_read; // writing the old cache entry to RAM
                    next_state = writeback; // move to writeback on replacement
                end
                
                else if (!tag_read.dirty && !(read_hit || write_hit) && (mhub.read_addr_valid || mhub.write_addr_valid)) 
                begin
                    next_state = allocate; // move to allocate to populate empty entry
                end
                else 
                begin
                    next_state = compare_tag; // default stay in compare_tag state
                end
            end
            
            allocate:
            begin
                ram.write_addr_valid = 0;
                mhub.read_addr_ready = 0;
                mhub.write_addr_ready = 0;
                mhub.read_data_valid = 0;
                ram.read_addr = (mhub.read_addr_valid) ? mhub.read_addr : mhub.write_addr;
                
                // Next Stage calculation
                if (!ram.read_data_valid) 
                begin
                    ram.read_addr_valid = 1;
                    next_state = allocate; // stay in allocate if response or memory isn't ready
                end
               
                else if (ram.read_data_valid) 
                begin
                    ram.read_addr_valid = 0; // disable reading from RAM while saving into the cache
                    from_ram = 1;
                    
                    // Set up request to write to cache tag module
                    tag_req.we = 1; // change request to a write
                    
                    // Set tag data to write to the module
                    tag_write.valid = 1;
                    tag_write.dirty = 0;
                    
                    // Set up request to write to cache data module
                    data_req.we = 1; // change request to a write
                    
                    // Set data to write to the module
                    data_write = ram.read_data;
                    next_state = compare_tag; // return to compare_tag when both response and memory are ready
                end
                
                else 
                begin
                    next_state = allocate; // default stay in allocate state
                end
            end
            
            writeback:
            begin
                ram.read_addr_valid = 0; // disable reading from the RAM
                ram.write_addr_valid = 1; // enable writing to the RAM
                mhub.read_addr_ready = 0; // don't want to read from the RAM in this stage
                mhub.write_addr_ready = 0; // data has not been written yet
                mhub.read_data_valid = 0; // not reading data from RAM in this stage
                ram.write_addr[31:12] = tag_read.tag;
                ram.write_addr[11:0] = (mhub.write_addr_valid) ? mhub.write_addr[11:0] : mhub.read_addr[11:0];                
                // Next Stage calculation
                if (!ram.write_addr_ready) 
                begin
                    next_state = writeback; // stay in writeback until the memory is ready
                end
                else if (ram.write_addr_ready)
                begin
                    tag_write.dirty = 0; // data that was in the cache is now aligned with what's in the memory
                    next_state = allocate; // go to allocate when memory is ready
                end
                
                else 
                begin
                    next_state = writeback; // default stay in writeback
                end
            end
            
            default:
            begin 
                next_state = compare_tag;     
            end  
        endcase
    end
	
	always_comb
	begin
	   tag_req.index = (mhub.read_addr_valid) ? mhub.read_addr[11:4] : mhub.write_addr[11:4]; // grab index from address field
	   tag_write.tag = (mhub.read_addr_valid) ? mhub.read_addr[TAG_MSB:TAG_LSB] : mhub.write_addr[TAG_MSB:TAG_LSB]; // acquire the tag from the address field
	   
	   data_req.index = (mhub.read_addr_valid) ? mhub.read_addr[11:4] : mhub.write_addr[11:4]; // grab index from address field
       be = mhub.strobe;
	   block_offset = (mhub.read_addr_valid) ? mhub.read_addr[3:2] : mhub.write_addr[3:2];
	end
	
    L1_cache_tag L1_tags(.*);
    L1_cache_data L1_data(.*);

endmodule
