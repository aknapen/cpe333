import cpu_types::*;

module ReservationStation(
   input MapTable MT, 
//   input RS_tag_type T1, T2, // register tags from MAP table 
   input RS_tag_type id, // tag of current reservation staion (self-identifier)
   input task_t DISPATCH_TASK,
   input RS_tag_type dest_res, // Reservation Station Selector
   input RS_tag_type CDB_tag, // tag from CDB
   input [31:0] CDB_val, // value from CDB
   input logic done, // FU tells RS when it's done
   
   output logic BUSY, // tell issue queue if RS is busy
   output logic [31:0] V1, V2, // value of operands for FU
   output logic V1_valid, V2_valid, // tell FU if operands are 
   output logic [4:0] rd_addr, // specify to FU where register result needs to go to
   output opcode_t OPCODE
);
    
    logic busy;
    RS_tag_type T1, T2;
    logic [31:0] v1, v2;
    RS_tag_type RD_tag;
    task_t curr_task;
    
    always_comb // Busy Calculation
    begin
        busy = 0;
        if (!done) busy = 1; // signal to issue queue that RS is ready
    end

    always_comb // Task Selection Calculation
    begin
        if (id == dest_res) curr_task = DISPATCH_TASK;
        if(done) curr_task = INVALID; // Need a way to clear the reservation station
    end
    
    always_comb
    begin
        RD_tag = MapTable[curr_task.rd].tag; // Don't know the syntax yet for accessing the maptable
        
        // Maps the initial value of V1 based on whether or not RS1 is in the Map Table
        if (MapTable[curr_task.rs1] == 0) // If the RS1 is not mapped in the Map Table go to Reg file or Immediate
        begin
            T1 = INVALID;
            V1_valid = 1;
            reg_rs1 = reg_file[rs1] // Don't know the syntax for getting this
            v1 = (curr_task.rs1_uder) ? reg_rs2 : curr_task.uimmediate;
        end
        else
        begin
            V1_valid = 0;
            t1 = MapTable[curr_task.rs1]
        end
        
        // Maps the initial value of V2 based on whether or not RS2 is in the Map Table
        if (MapTable[curr_task.rs2] == 0) // If the RS2 is not mapped in the Map Table go to Reg file or Immediate
        begin
            T2 = INVALID; // So we don't catch broadcast we don't want
            V2_valid = 1; // We got a valid value from the reg file
            reg_rs1 = reg_file[rs2] // Don't know the syntax for getting this
            v2 = (curr_task.rs2_used) ? reg_rs2 : curr_task.uimmediate; // Not sure about this one
        end
        else // If RS2 is in the map table we set T2 alias and make sure V2 is invalid
        begin
            V2_valid = 0;
            t2 = MapTable[curr_task.rs2]
        end
        
        
        if(CDB_tag == T1)
        begin
            V1_valid = 1;
            v1 = CDB_value;
        end 
        
        if(CDB_tag == T2)
        begin
            V2_valid = 1;
            v2 = CDB_value;
        end 
        
    end
    
    assign V1 = v1;
    assign V2 = v2;
    assign BUSY = busy;
    assign OPCODE = curr_task.opcode;
    
endmodule