import cpu_types::*;

module ReservationStation #(parameter RS_TAG = INVALID) (
   input task_t DISPATCH_TASK,
   input RS_tag_type dest_RS, // Reservation Station Selector
   input RS_tag_type T1, T2, T3, // register tags from MAP table
//   input RS_tag_type CDB_tag, // tag from CDB
//   input [31:0] CDB_val, // value from CDB
   input cdb_t cdb_in, // tag-value pair broadcast on CDB
   input done, // FU tells RS when it's done
   input [31:0] rs2_data,
   input [31:0] A,
   input [31:0] B,
   
   output BUSY, // tell issue queue if RS is busy
   output [31:0] V1, V2, V3, // value of operands for FU
   output V1_valid, V2_valid, V3_valid, // tell FU if operands are 
   output RS_tag_type rd_tag, // specify to FU where register result needs to go to
//   output [31:0] rs2_data, // need to send rs2 data to the store unit
   output [2:0] mem_type,
   output [3:0] alu_fun // output function to ALU
);
    
    // intermediate outputs
    logic busy;
    logic [31:0] v1, v2, v3;
    logic v1_valid, v2_valid, v3_valid;
    task_t curr_task;
    
//    always_comb // Busy Calculation
//    begin
//        busy = 0;
//        if (!done) busy = 1; // signal to issue queue that RS is ready
//    end
    always_comb // Task Selection Calculation
    begin
    
        if (done)
        begin
            busy = 0;
        end
        
        if (RS_TAG == dest_RS)
        begin
            curr_task = DISPATCH_TASK;
            busy = 1;
        end
        
        
    end
    
    always_comb // setting V1
    begin
        v1_valid = 0;        
        // Maps the initial value of V1 based on whether or not RS1 is in the Map Table
        if (T1 == INVALID && RS_TAG == dest_RS) // If the RS1 is not mapped in the Map Table go to Reg file or Immediate
        begin
            v1_valid = 1; // use input value A from incoming task
            v1 = A;
        end
        else if(cdb_in.tag == T1 && T1 != INVALID)
        begin
            v1_valid = 1;
            v1 = cdb_in.data;
        end
        if (!busy) v1_valid = 0;
    end
    
    always_comb // setting V2
    begin
        v2_valid = 0;        
        // Maps the initial value of V2 based on whether or not RS2 is in the Map Table
        if (T2 == INVALID && RS_TAG == dest_RS) // If the RS2 is not mapped in the Map Table go to Reg file or Immediate
        begin
            v2_valid = 1; // use input value B from incoming task
            v2 = B;
        end
        else if(cdb_in.tag == T2 && T2 != INVALID)
        begin
            v2_valid = 1;
            v2 = cdb_in.data;
        end
        if (!busy) v2_valid = 0;
    end
    
    always_comb // setting V3
    begin
        v3_valid = 0;        
        // Maps the initial value of V2 based on whether or not RS2 is in the Map Table
        if (T3 == INVALID && RS_TAG == dest_RS) // If the RS2 is not mapped in the Map Table go to Reg file or Immediate
        begin
            v3_valid = 1; // use input value B from incoming task
            v3 = rs2_data;
        end
        else if(cdb_in.tag == T3 && T3 != INVALID)
        begin
            v3_valid = 1;
            v3 = cdb_in.data;
        end
        if (!busy) v3_valid = 0;
    end
    
    assign V1 = v1;
    assign V1_valid = v1_valid;
    assign V2 = v2;
    assign V2_valid = v2_valid;
    assign V3_valid = v3_valid;
    assign V3 = v3;
    assign rd_tag = RS_TAG;
    assign BUSY = busy;
    assign OPCODE = curr_task.opcode;
    assign alu_fun = curr_task.alu_fun; // ALU FU needs alu fun
    assign mem_type = curr_task.mem_type; // STORE/LOAD FUs need mem_type
endmodule