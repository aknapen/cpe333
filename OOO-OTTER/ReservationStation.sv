import cpu_types::*;

module ReservationStation(
   input RS_tag_type T1, T2, // register tags from MAP table 
   input RS_tag_type id, // tag of current reservation staion (self-identifier)
   input task_t DISPATCH_TASK,
   input RS_tag_type dest_res,
   input RS_tag_type CDB_tag, // tag from CDB
   input [31:0] CDB_val, // value from CDB
   input done, // FU tells RS when it's done
   
   output BUSY, // tell issue queue if RS is busy
   output [31:0] V1, V2, // value of operands for FU
   output V1_valid, V2_valid, // tell FU if operands are 
   output [4:0] rd_addr, // specify to FU where register result needs to go to
);
    
    logic busy;
    logic [31:0] v1, v2;
    
    always_comb
    begin
        busy = 0;
        if (!done) busy = 1; // signal to issue queue that RS is ready
    end

    always_comb
    begin
        if (
    end
    
    assign V1 = v1;
    assign V2 = v2;
    assign BUSY = busy;
endmodule