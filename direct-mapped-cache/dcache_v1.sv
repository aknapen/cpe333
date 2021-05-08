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
    logic valid;
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
    
    
	// tag storage  
	
	// If tag_reg.we then tags[tag_req.index] <= tag_write
	// tag_read = tags[tag_req.index]
	// incluces valid and dirty bits
	// async read, sync write
	
	
endmodule


module dcache(
    input clk, RESET,
    axi_bus_rw.device mhub,
    axi_bus_rw.controller ram
    );
    
//            // From RAM
//	        input read_addr_ready, read_data, read_data_valid,
//			input  write_addr_ready, write_resp_valid, //write_data_ready,
//			// TO RAM
//			output read_addr, read_addr_valid, //read_data_ready,
//			output write_addr, write_addr_valid, write_data, //write_data_valid,
//			output size, lu, strobe);
			
//            // From Memory Hub
//			input read_addr, read_addr_valid, //read_data_ready, 
//			input write_addr, write_addr_valid, write_data, //write_data_valid,
//			input size, lu, strobe,
//			// To Memory Hub
//			output read_addr_ready, read_data, read_data_valid,
//			output  write_addr_ready, write_resp_valid);       //write_data_ready,
			
			
    // Inputs to the cache
    // Will use mhub and ram output signals
    cpu_req_type cpu_req;     //CPU->cache
    mem_data_type mem_data;   //memory->cache
    
    // Outputs from the cache
    // Will set mhub and ram input signals
    mem_req_type mem_req;    //cache->memory
    cpu_result_type cpu_res;  //cache->CPU
    
       

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
    
    cpu_result_type next_cpu_res;
    
//=================== Establish inputs and outputs to and from the cache ==============================

    // Get the necessary data from the mhub (mhub => cpu_req)
//    assign cpu_req.addr = (mhub.read_addr_valid) ? mhub.read_addr : mhub.write_addr;
//    assign cpu_req.data = mhub.write_data;
//    assign cpu_req.rw = (mhub.write_addr_valid) ? 1 : 0;
//    assign cpu_req.valid = (mhub.read_addr_valid || mhub.write_addr_valid);
	   
    // Send the necessary data to the mhub (cpu_res => mhub)
//    assign mhub.read_data = cpu_res.data;
//    assign mhub.read_addr_ready = cpu_res.ready;
//    assign mhub.read_data_valid = !cpu_req.rw; // read data only valid if incoming request was a read?
//    assign mhub.write_addr_ready = cpu_res.ready;
//    assign mhub.write_resp_valid = cpu_req.rw; // write resp only valid if incoming request was a write?
    
    // Send the necessary data to the RAM (mem_req => ram)
//    assign ram.read_addr = mem_req.addr;
//    assign ram.write_addr = mem_req.addr;
//    assign ram.write_data = mem_req.data;
//    assign ram.read_addr_valid = !mem_req.rw && mem_req.valid;
//    assign ram.write_addr_valid = mem_req.rw && mem_req.valid;
//    assign ram.strobe = be; // I think we were supposed to be given logic for these two signals
//    assign ram.size = block_offset;
    
    // Get the necessary data from the RAM (ram => mem_data)
//    assign mem_data.data = ram.read_data;
//    assign mem_data.ready = (mem_req.rw) ? ram.write_addr_ready : ram.read_addr_ready;
//    assign mem_data.valid = (mem_req.rw) ? ram.write_resp_valid : ram.read_data_valid;

//=======================================================================================================
	
	typedef enum {idle, compare_tag, allocate, writeback} cache_state_type;
    cache_state_type state, next_state;
    
	//FSM for Cache Controller
	
	assign write_hit = ((mhub.write_addr[TAG_MSB:TAG_LSB] == tag_read.tag) && mhub.write_addr_valid) && tag_read.valid;
	assign read_hit = ((mhub.read_addr[TAG_MSB:TAG_LSB] == tag_read.tag) && mhub.read_addr_valid) && tag_read.valid;
	
//    initial begin // set FSM to Compare Stage initially
//        state = compare_tag;
//    end
    
    always_ff @(posedge clk) begin
        state <= next_state;
    end
    
    always_comb 
    begin
        case(state)
            compare_tag:
            begin
                mhub.read_addr_ready = 1;
                mhub.write_addr_ready = 1;
                mhub.read_data_valid = 0;
                mhub.write_resp_valid = 0;
                
                // Next Stage calculation
                if (write_hit) // successful write
                begin
                    tag_write.dirty = 1; // data in cache no longer corresponds to data in memory
                    mhub.write_resp_valid = 1;
                    from_ram = 0; // writing from CPU data, not from RAM
                    next_state = compare_tag; // stay in compare_tag on a successful read or write to cache
                end
                
                else if (read_hit) // successful read
                begin
//                    mhub.read_data = data_read[127-((3-block_offset)*WORD_WIDTH):block_offset*WORD_WIDTH]; // grab the correct word from the block read
                    mhub.read_data = data_read>>(block_offset*WORD_WIDTH);
                    mhub.read_data_valid = 1;
                end
                
                else if (tag_read.dirty && !(read_hit && write_hit)) 
                begin
                    ram.write_data = data_read; // writing the old cache entry to RAM
                    next_state = writeback; // move to writeback on replacement
                end
                
                else if (!tag_read.dirty && !(read_hit && write_hit)) next_state = allocate; // move to allocate to populate empty entry
                
                else next_state = compare_tag; // default stay in compare_tag state
            end
            
            allocate:
            begin
                ram.write_addr_valid = 0;
                mhub.read_addr_ready = 0;
                mhub.write_addr_ready = 0;
                mhub.read_data_valid = 0;
                ram.read_addr = mhub.read_addr;
                
                // Next Stage calculation
                if (!ram.read_addr_ready) 
                begin
                    ram.read_addr_valid = 1;
                    next_state = allocate; // stay in allocate if response or memory isn't ready
                end
               
                else if (ram.read_addr_ready) 
                begin
                    ram.read_addr_valid = 0; // disable reading from RAM while saving into the cache
                    from_ram = 1;
                    tag_write.tag = (tag_req.we) ? mhub.write_addr[TAG_MSB:TAG_LSB] : mhub.read_addr[TAG_MSB:TAG_LSB];
                    tag_write.valid = 1;
                    tag_write.dirty = 0;
                    next_state = compare_tag; // return to compare_tag when both response and memory are ready
                end
                
                else next_state = allocate; // default stay in allocate state
            end
            
            writeback:
            begin
                ram.read_addr_valid = 0; // disable reading from the RAM
                ram.write_addr_valid = 1; // enable writing to the RAM
                mhub.read_addr_ready = 0; // don't want to read from the RAM in this stage
                mhub.write_addr_ready = 0; // data has not been written yet
                mhub.read_data_valid = 0; // not reading data from RAM in this stage
                ram.write_addr = {tag_write.tag, mhub.write_addr[11:0]};
                
                // Next Stage calculation
                if (!ram.write_addr_ready) next_state = writeback; // stay in writeback until the memory is ready
                
                else if (ram.write_addr_ready)
                begin
                    tag_write.dirty = 0; // data that was in the cache is now aligned with what's in the memory
                    next_state = allocate; // go to allocate when memory is ready
                end
                
                else next_state = writeback; // default stay in writeback
            end
            
            default: next_state = compare_tag;       
        endcase
    end
	
    assign tag_req.index = (mhub.read_addr_valid) ? mhub.read_addr[11:2] : mhub.write_addr[11:2]; // grab index from address field
    assign tag_req.we = (mhub.write_addr_valid) ? 1 : 0; // only enable write on a write request
    assign tag_write.tag = (mhub.read_addr_valid) ? mhub.read_addr[TAG_MSB:TAG_LSB] : mhub.write_addr[TAG_MSB:TAG_LSB]; // acquire the tag from the address field
    L1_cache_tag L1_tags(.*);

    assign data_req.index = (mhub.read_addr_valid) ? mhub.read_addr[11:2] : mhub.write_addr[11:2]; // grab index from address field
    assign data_req.we = (mhub.write_addr_valid) ? 1 : 0; // only enable write on a write request
    assign data_write = (from_ram) ? ram.read_data : mhub.write_data;
    assign be = (mhub.read_addr_valid) ? mhub.read_addr[3:0] : mhub.write_addr[3:0]; // be is the concatenation of the block and byte offset fields
    assign block_offset = (mhub.read_addr_valid) ? mhub.read_addr[3:2] : mhub.write_addr[3:2];
    L1_cache_data L1_data(.*);

endmodule
