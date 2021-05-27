`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/05/2019 12:17:57 AM
// Design Name: 
// Module Name: registerFile
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


module OTTER_registerFile(Read1_0, Read1_1,Read2_0,Read2_1,WriteReg,WriteData,RegWrite,Data1_0, Data1_1,Data2_0,Data2_1,clock);
    input [4:0] Read1_0, Read1_1,Read2_0, Read2_1,WriteReg; //the register numbers to read or write
    input [31:0] WriteData; //data to write
    input RegWrite, //the write control
        clock;  // the clock to trigger write
    output logic [31:0] Data1_0, Data1_1, Data2_0, Data2_1; // the register values read
    logic [31:0] RF [31:0]; //32 registers each 32 bits long
    
    always_comb // read in rs1 values
    begin
        if(Read1_0==0) Data1_0 = 0;
        else Data1_0 = RF[Read1_0];
        
        if(Read1_1==0) Data1_1 = 0;
        else Data1_1 = RF[Read1_1];
    end
    
    always_comb // read in rs2 values
    begin
        if(Read2_0==0) Data2_0 =0;
        else Data2_0 = RF[Read2_0];
        
        if(Read2_1==0) Data2_1 =0;
        else Data2_1 = RF[Read2_1];
    end
    
    always@(negedge clock) begin // write the register with the new value if Regwrite is high
        if(RegWrite && WriteReg!=0) RF[WriteReg] <= WriteData;
        
    end
 endmodule

