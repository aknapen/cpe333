import cpu_types::*;

module ReservationStation #(parameter RS_TAG = INVALID) (
   input task_t DISPATCH_TASK,
   input RS_tag_type dest_RS, // Reservation Station Selector
   input RS_tag_type T1, T2, T3, // register tags from MAP table
   input RS_tag_type CDB_tag, // tag from CDB
   input [31:0] CDB_val, // value from CDB
   input done, // FU tells RS when it's done
   
   output BUSY, // tell issue queue if RS is busy
   output [31:0] V1, V2, V3, // value of operands for FU
   output V1_valid, V2_valid, V3_valid, // tell FU if operands are 
   output RS_tag_type rd_tag, // specify to FU where register result needs to go to
   output [31:0] rs2_data, // need to send rs2 data to the store unit
   output [2:0] mem_type,
   output alu_fun // output function to ALU
);
    
    // intermediate outputs
    logic busy;
    logic [31:0] v1, v2, v3;
    logic v1_valid, v2_valid, v3_valid;
    task_t curr_task;
    
    always_comb // Busy Calculation
    begin
        busy = 0;
        if (!done) busy = 1; // signal to issue queue that RS is ready
    end

    always_comb // Task Selection Calculation
    begin
        if (RS_TAG == dest_RS) curr_task = DISPATCH_TASK;
    end
    
    always_comb // setting V1
    begin
        v1_valid = 0;        
        // Maps the initial value of V1 based on whether or not RS1 is in the Map Table
        if (T1 == INVALID) // If the RS1 is not mapped in the Map Table go to Reg file or Immediate
        begin
            v1_valid = 1; // use input value A from incoming task
            v1 = curr_task.A;
        end
        else if(CDB_tag == T1)
        begin
            v1_valid = 1;
            v1 = CDB_value;
        end
    end
    
    always_comb // setting V2
    begin
        v2_valid = 0;        
        // Maps the initial value of V2 based on whether or not RS2 is in the Map Table
        if (T2 == INVALID) // If the RS2 is not mapped in the Map Table go to Reg file or Immediate
        begin
            v2_valid = 1; // use input value B from incoming task
            v2 = curr_task.B;
        end
        else if(CDB_tag == T2)
        begin
            v2_valid = 1;
            v2 = CDB_value;
        end
    end
    
    always_comb // setting V3
    begin
        v3_valid = 0;        
        // Maps the initial value of V2 based on whether or not RS2 is in the Map Table
        if (T3 == INVALID) // If the RS2 is not mapped in the Map Table go to Reg file or Immediate
        begin
            v3_valid = 1; // use input value B from incoming task
            v3 = curr_task.rs2_data;
        end
        else if(CDB_tag == T3)
        begin
            v3_valid = 1;
            v3 = CDB_value;
        end
    end
    
    assign V1 = v1;
    assign V1_valid = v1_valid;
    assign V2 = v2;
    assign V2_valid = v2_valid;
    
    assign BUSY = busy;
    assign OPCODE = curr_task.opcode;
    assign alu_fun = curr_task.alu_fun; // ALU FU needs alu fun
    assing rs2_data = curr_task.rs2_data; // STORE FU needs RS2 data
    assign mem_type = curr_task.mem_type; // STORE/LOAD FUs need mem_type
endmodule