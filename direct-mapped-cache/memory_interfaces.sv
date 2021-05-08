`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Keefe Johnson
// 		Joseph Callenes
// Create Date: 02/06/2020 06:40:37 PM
// Updated Date: 06/20/2020 01:30:00 AM
// Design Name: 
// Module Name: memory_interfaces
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - Updated for new interface (similar to AXI-lite)
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


package memory_bus_sizes;
    // user-defined
    parameter ADDR_WIDTH = 32;  // bits
    parameter DATA_WIDTH = 32;  // bits
    parameter WORD_WIDTH = 32;
    parameter WORD_SIZE = DATA_WIDTH / 8;  //bytes  /STRB -strobe
    parameter MMIO_START_ADDR = 'h11000000;
    //for blocked memory
    parameter BLOCK_SIZE =16;   //bytes, power of 2
    parameter BLOCK_WIDTH = BLOCK_SIZE * 8;
    parameter WORD_ADDR_LSB = $clog2(WORD_SIZE);
    parameter BLOCK_ADDR_LSB = $clog2(BLOCK_SIZE);
endpackage
import memory_bus_sizes::*;


interface axi_bus_ro();		// similar to AXI-lite bus - read only
	logic [DATA_WIDTH-1:0] read_data;
	logic read_data_valid;
	//logic read_data_ready;

	logic [ADDR_WIDTH-1:0] read_addr;
	logic read_addr_valid;
	logic read_addr_ready;

	logic [WORD_SIZE-1:0]strobe;  //byte enable

	modport controller (
			output read_addr, read_addr_valid, //read_data_ready, 
			input read_addr_ready, read_data, read_data_valid);

	modport device (
			input read_addr, read_addr_valid, //read_data_ready, 
			output read_addr_ready, read_data, read_data_valid);
endinterface

interface axi_bus_rw #(parameter WIDTH=32);		// similar to AXI-lite bus - read/write
	logic [WIDTH-1:0] read_data;
	logic read_data_valid;
	//logic read_data_ready;     

	logic [ADDR_WIDTH-1:0] read_addr;
	logic read_addr_valid;
	logic read_addr_ready;

	logic [WIDTH-1:0] write_data;
	//logic write_data_valid;
	//logic write_data_ready;
	logic write_resp_valid;    //write response valid -- for now, assume response is simply an acknowledgment of write completion

	logic [ADDR_WIDTH-1:0] write_addr;
	logic write_addr_valid;
	logic write_addr_ready;

	logic [WORD_SIZE-1:0]strobe;  //byte enable
    logic [1:0] size;
	logic lu;  //unsigned number

	modport controller (
	        input read_addr_ready, read_data, read_data_valid,
			input  write_addr_ready, write_resp_valid, //write_data_ready,
			output read_addr, read_addr_valid, //read_data_ready,
			output write_addr, write_addr_valid, write_data, //write_data_valid,
			output size, lu, strobe);
			

	modport device (
			input read_addr, read_addr_valid, //read_data_ready, 
			input write_addr, write_addr_valid, write_data, //write_data_valid,
			input size, lu, strobe,
			output read_addr_ready, read_data, read_data_valid,
			output  write_addr_ready, write_resp_valid);       //write_data_ready,
endinterface

// empty module just to force Vivado to show this file in source hierarchy
module memory_interfaces(); endmodule
