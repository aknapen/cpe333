`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/12/2021 04:42:06 PM
// Design Name: 
// Module Name: Register
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


module Register(
    input EN,
    input [63:0] DIN,
    input CLK,
    output [63:0] DOUT,
    input RST
    );
    
    logic [63:0] reg_data; // what is being held in the register
    
    always_comb
        DOUT = reg_data; // always allow for reading data from the register
    
    always_ff @(posedge CLK) 
    begin
        if (RST == 1) reg_data = 0; // resets value in the register
        if (EN == 1) reg_data = DIN; // only allow for data to be written to the register on CLK edges and if enable is set
    end
endmodule
