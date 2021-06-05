`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/27/2021 09:34:27 AM
// Design Name: 
// Module Name: IssueQueue
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

module IssueQueue(
    input CLK,
    input task_t TASK_0,
    input task_t TASK_1,
    input [5:0] rs_busy,
    output task_t DISPATCH_TASK,
    output RS_tag_type dest_rs,
    output FULL
    );
    
    task_t task_queue [$:15];
    task_t dispatch_task;
    logic full;
    logic stall; 
    logic [5:0] busy;
    RS_tag_type rs;
    
    assign busy = rs_busy;
    
    always_ff @(posedge CLK) // Issue Stage
    begin
        full = 0;
        if (task_queue.size() == 15) full = 1;
        else
        begin // insert new tasks into issue queue
            task_queue.push_back(TASK_0);
            task_queue.push_back(TASK_1);
        end
    end
    
    always_ff @(posedge CLK) // Dispatch stage
    begin
        stall = 0;
        rs = INVALID;
        case (task_queue[0].opcode) // peak top of queue
            STORE:
            begin // assign to available store RS or stall
                if (!busy[0]) rs = STORE_1;
                else if (!busy[1]) rs = STORE_2;
                else stall = 1;
            end
            
            LOAD:
            begin // assign to available load RS or stall
                if (!busy[3]) rs = LOAD_1;
                else if (!busy[4]) rs = LOAD_2;
                else stall = 1;
            end
           
            default:
            begin // assign to available ALU RS or stall
                if (!busy[4]) rs = ALU_1;
                else if (!busy[5]) rs = ALU_2;
                else stall = 1;
            end
        endcase
        
        if (!stall) dispatch_task = task_queue.pop_front();  
    end
    
    assign DISPATCH_TASK = dispatch_task;
    assign dest_rs = rs;
    assign FULL = full;
    
endmodule
