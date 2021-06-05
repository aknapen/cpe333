`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: Pipelined OTTER CPU
// Module Name: OTTER_CPU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.10 - (Keefe Johnson, 1/14/2020) Added serial programmer.
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

 package cpu_types;
       typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011,
           NOP      = 7'b0000000
       } opcode_t;
       
       // struct for storing a given pipeline stage's instruction and PC value\
       typedef struct packed{
           opcode_t opcode;
           logic [4:0] rs1_addr;
           logic [4:0] rs2_addr;
           logic [31:0] rs2_val; // value of rs2 needed for stores
           logic [4:0] rd_addr;
           logic rs1_used;
           logic rs2_used;
           logic rd_used;
           logic [3:0] alu_fun;
           logic memWrite;
           logic memRead2;
           logic regWrite;
           logic [1:0] rf_wr_sel;
           logic [2:0] mem_type;  //sign, size
           logic ld_haz;
           logic [31:0] pc;
           logic invalid;
           logic br_taken;
       } instr_t;
       
       typedef struct packed {
           logic [31:0] A;
           logic [31:0] B;
           opcode_t opcode;
           logic [4:0] rd_addr;
           logic rd_used;
           logic [4:0] rs1_addr;
           logic rs1_used;
           logic [4:0] rs2_addr;
           logic rs2_used;
           logic [31:0] rs2_data;
           logic [3:0] alu_fun;
           logic [2:0] mem_type;
           logic [1:0] wb_sel;
       } task_t;

       typedef enum {
          INVALID,
          STORE_1,
          STORE_2,
          LOAD_1,
          LOAD_2,
          ALU_1,
          ALU_2
       } RS_tag_type;  
       
       typedef struct packed {
          RS_tag_type tag;
          logic busy;
       } MTEntry_type;
       
       typedef struct packed {
          RS_tag_type tag;
          logic [31:0] data;
       } cdb_t;
       
   endpackage

module OTTER_MCU(input CLK,
                input INTR,
                input EXT_RESET,  // CHANGED RESET TO EXT_RESET FOR PROGRAMMER
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR,
                input PROG_RX,  // ADDED PROG_RX FOR PROGRAMMER
                output PROG_TX  // ADDED PROG_TX FOR PROGRAMMER
);           
    
    import cpu_types::*;
      
    // ************************ BEGIN PROGRAMMER ************************ 
    parameter RS_TAG = INVALID;
    
    wire RESET;
    wire [31:0] s_prog_ram_addr;
    wire [31:0] s_prog_ram_data;
    wire s_prog_ram_we;
    wire s_prog_mcu_reset;
    wire [31:0] mem_addr_after;
    wire [31:0] mem_data_after;
    wire [1:0] mem_size_after;
    wire mem_sign_after;
    wire mem_we_after;

    programmer #(.CLK_RATE(50), .BAUD(115200), .IB_TIMEOUT(200),
                 .WAIT_TIMEOUT(500))
        programmer(.clk(CLK), .rst(EXT_RESET), .srx(PROG_RX), .stx(PROG_TX),
                   .mcu_reset(s_prog_mcu_reset), .ram_addr(s_prog_ram_addr),
                   .ram_data(s_prog_ram_data), .ram_we(s_prog_ram_we));

    // ************************ END PROGRAMMER ************************ 

    instr_t DE_EX_instr;
    instr_t EX_MEM_instr;
    instr_t MEM_WB_instr;
        
    logic [31:0] IF_ID_pc_0, IF_ID_pc_1; // transmits the PC between the IF and ID stages
    logic [31:0] WB_rfIn; // connect to din for the reg file
    logic jb_taken; // indicates if a jump or branch has been taken
    
    logic [31:0] MEM_aluResult; // holds the ALU result value inside the memory stage
    logic [31:0] WB_aluResult; // holds the ALU result value inside the writeback stage
    
    logic [31:0] mem_data; // MEM_DOUT2 wire
    
    wire intTaken;

    //=======================  BEGIN FETCH STAGE ===========================//

    // Inputs to IF module
    logic [31:0] jalr_pc, branch_pc, jump_pc; 
    logic [1:0] pc_source;
    logic ld_haz; // used to detect a load-use hazard

    // Outputs from IF module
    logic [31:0] fetch_pc_0, fetch_pc_1;
    logic memRead1;

    InstructionFetch IF(.*); 

    //=======================  END FETCH STAGE ===========================//

    // PC REGISTER
    always_ff @(posedge CLK) // transfers PC from fetch to decode
    begin
        IF_ID_pc_0 <= fetch_pc_0; // delay PC from fetch stage
        IF_ID_pc_1 <= fetch_pc_1;
    end
    
    
//======================= BEGIN DECODE STAGE ===========================//

    logic [31:0] IR_0, IR_1;
    task_t TASK_0, TASK_1;
    
    InstructionDecode ID(.*);


//======================= END DECODE STAGE ===========================//
    logic [5:0] rs_busy; // busy bits generated by reservation stations
    task_t DISPATCH_TASK;
    RS_tag_type dest_rs;
    logic FULL; // stall previous stages when issue queue is full
    
    // Issue Queue
    IssueQueue IQ(.*);
    
//======================= BEGIN EXECUTE STAGE ===========================//

    logic regWrite;
    logic [4:0] regWrite_addr;
    RS_tag_type T1, T2, T3;
    cdb_t cdb_data;
    
    // Map Table
    MapTable MT(.task_opcode(DISPATCH_TASK.opcode), .rs1_used(DISPATCH_TASK.rs1_used), .rs2_used(DISPATCH_TASK.rs2_used),
                .rs1_addr(DISPATCH_TASK.rs1_addr), .rs2_addr(DISPATCH_TASK.rs2_addr), .rd_addr(DISPATCH_TASK.rd_addr),
                .cdb_in(cdb_data), .issue_tag(dest_rs), .T1(T1), .T2(T2), .T3(T3), .reg_data(WB_rfIn), .writeReg_addr(regWrite_addr),
                .reg_valid(regWrite), .CLK(CLK)); // Maps Register -> Reservation Station
    
    
    // Reservation Stations / Functional Units
        
    //============== Load 1==============//
    logic L1_done, L1_busy, L1_V1_valid, L1_V2_valid, L1_V3_valid;
    logic [31:0] L1_V1, L1_V2, L1_V3;
    RS_tag_type L1_rd_tag;
//    logic [31:0] L1_rs2_data;
    logic [2:0] L1_mem_type;
    logic [3:0] L1_alu_fun;
    ReservationStation #(LOAD_1) RS_Load1 (.DISPATCH_TASK(DISPATCH_TASK), .dest_RS(dest_rs), .T1(T1), .T2(T2), .T3(T3),
                                           .cdb_in(cdb_data), .done(L1_done), .BUSY(L1_busy), .V1(L1_V1),
                                           .V2(L1_V2), .V3(L1_V3), .V1_valid(L1_V1_valid), .V2_valid(L1_V2_valid), .V3_valid(L1_V3_valid),
                                           .rd_tag(L1_rd_tag), .mem_type(L1_mem_type), .alu_fun(L1_alu_fun));
    logic [31:0] L1_mem_data_in;
    RS_tag_type L1_CDB_tag;
    logic [31:0] L1_CDB_val;
    logic [31:0] L1_mem_addr_2;
    logic L1_memRead;
    logic L1_mem_sign;
    logic [1:0] L1_mem_size;
    LoadUnit Load1(.V1(L1_V1), .V2(L1_V2), .V1_valid(L1_V1_valid), .V2_valid(L1_V2_valid), .rd_tag(L1_rd_tag), .mem_type(L1_mem_type), 
                   .mem_resp(L1_mem_resp), .mem_resp_valid(L1_mem_resp_valid), .mem_data_in(L1_mem_data_in), .MEM_READ(L1_memRead), .CDB_val(L1_CDB_val), 
                   .CDB_tag(L1_CDB_tag), .done(L1_done), .MEM_ADDR2(L1_mem_addr_2), .MEM_SIGN(L1_mem_sign), .MEM_SIZE(L1_mem_size));
    
    
    //============== Load 2==============//
    logic L2_done, L2_busy, L2_V1_valid, L2_V2_valid, L2_V3_valid;
    logic [31:0] L2_V1, L2_V2, L2_V3;
    RS_tag_type L2_rd_tag;
//    logic [31:0] L2_rs2_data;
    logic [2:0] L2_mem_type;
    logic [3:0] L2_alu_fun;
    ReservationStation #(LOAD_2) RS_Load2(.DISPATCH_TASK(DISPATCH_TASK), .dest_RS(dest_rs), .T1(T1), .T2(T2), .T3(T3),
                                           .cdb_in(cdb_data), .done(L2_done), .BUSY(L2_busy), .V1(L2_V1),
                                           .V2(L2_V2), .V3(L2_V3), .V1_valid(L2_V1_valid), .V2_valid(L2_V2_valid), .V3_valid(L2_V3_valid),
                                           .rd_tag(L2_rd_tag), .mem_type(L2_mem_type), .alu_fun(L2_alu_fun));
                                           
    logic [31:0] L2_mem_data_in;
    RS_tag_type L2_CDB_tag;
    logic [31:0] L2_CDB_val;
    logic [31:0] L2_mem_addr_2;
    logic L2_memRead;
    logic L2_mem_sign;
    logic [1:0] L2_mem_size;
    LoadUnit Load2(.V1(L2_V1), .V2(L2_V2), .V1_valid(L2_V1_valid), .V2_valid(L2_V2_valid), .rd_tag(L2_rd_tag), .mem_type(L2_mem_type), 
                   .mem_resp(L2_mem_resp), .mem_resp_valid(L2_mem_resp_valid), .mem_data_in(L2_mem_data_in), .MEM_READ(L2_memRead), .CDB_val(L2_CDB_val), 
                   .CDB_tag(L2_CDB_tag), .done(L2_done), .MEM_ADDR2(L2_mem_addr_2), .MEM_SIGN(L2_mem_sign), .MEM_SIZE(L2_mem_size));
    
    
    //============== Store 1==============//
    logic S1_done, S1_busy, S1_V1_valid, S1_V2_valid, S1_V3_valid;
    logic [31:0] S1_V1, S1_V2, S1_V3;
    RS_tag_type S1_rd_tag;
//    logic [31:0] S1_rs2_data;
    logic [2:0] S1_mem_type;
    logic [3:0] S1_alu_fun;
    ReservationStation #(STORE_1) RS_Store1(.DISPATCH_TASK(DISPATCH_TASK), .dest_RS(dest_rs), .T1(T1), .T2(T2), .T3(T3),
                                           .cdb_in(cdb_data), .done(S1_done), .BUSY(S1_busy), .V1(S1_V1),
                                           .V2(S1_V2), .V3(S1_V3), .V1_valid(S1_V1_valid), .V2_valid(S1_V2_valid), .V3_valid(S1_V3_valid),
                                           .rd_tag(S1_rd_tag), .mem_type(S1_mem_type), .alu_fun(S1_alu_fun));
    
    logic [4:0] S1_mem_addr_2;
    logic S1_mem_write;
    logic [31:0] S1_mem_write_data;
    logic S1_mem_sign;
    logic [1:0] S1_mem_size;
    StoreUnit Store1(.V1(S1_V1), .V2(S1_V2), .V3(S1_V3), .V1_valid(S1_V1_valid), .V2_valid(S1_V2_valid), .V3_valid(S1_V3_valid), .rd_tag(S1_rd_tag), 
                     .mem_type(S1_mem_type), .mem_resp(S1_mem_resp), .mem_resp_valid(S1_mem_resp_valid), .done(S1_done), .MEM_ADDR2(S1_mem_addr_2),
                     .MEM_WRITE(S1_mem_write), .MEM_WRITE_DATA(S1_mem_write_data), .MEM_SIGN(S1_mem_sign), .MEM_SIZE(S1_mem_size));
    
    
    //============== Store 2==============//
    logic S2_done, S2_busy, S2_V1_valid, S2_V2_valid, S2_V3_valid;
    logic [31:0] S2_V1, S2_V2, S2_V3;
    RS_tag_type S2_rd_tag;
//    logic [31:0] S2_rs2_data;
    logic [2:0] S2_mem_type;
    logic [3:0] S2_alu_fun;
    ReservationStation #(STORE_2) RS_Store2(.DISPATCH_TASK(DISPATCH_TASK), .dest_RS(dest_rs), .T1(T1), .T2(T2), .T3(T3),
                                           .cdb_in(cdb_data), .done(S2_done), .BUSY(S2_busy), .V1(S2_V1),
                                           .V2(S2_V2), .V3(S2_V3), .V1_valid(S2_V1_valid), .V2_valid(S2_V2_valid), .V3_valid(S2_V3_valid),
                                           .rd_tag(S2_rd_tag), .mem_type(S2_mem_type), .alu_fun(S2_alu_fun));
                                           
    logic [4:0] S2_mem_addr_2;
    logic S2_mem_write;
    logic [31:0] S2_mem_write_data;
    logic S2_mem_sign;
    logic [1:0] S2_mem_size;
    StoreUnit Store2(.V1(S2_V1), .V2(S2_V2), .V3(S2_V3), .V1_valid(S2_V1_valid), .V2_valid(S2_V2_valid), .V3_valid(S2_V3_valid), .rd_tag(S2_rd_tag), 
                     .mem_type(S2_mem_type), .mem_resp(S2_mem_resp), .mem_resp_valid(S2_mem_resp_valid), .done(S2_done), .MEM_ADDR2(S2_mem_addr_2),
                     .MEM_WRITE(S2_mem_write), .MEM_WRITE_DATA(S2_mem_write_data), .MEM_SIGN(S2_mem_sign), .MEM_SIZE(S2_mem_size));
    
    
    //============== OTTER MEMORY ==============//
    OTTER_mem_byte #(14) memory(.MEM_CLK(CLK),.MEM_ADDR1_0(fetch_pc_0), .MEM_ADDR1_1(fetch_pc_1), .MEM_READ_1(memRead1), .MEM_DOUT1_0(IR_0), .MEM_DOUT1_1(IR_1), // Instruction fetch
    
                        .MEM_ADDR2_L1(L1_mem_addr_2), .MEM_DOUT2_L1(L1_mem_data_in),  .MEM_READ2_L1(L1_memRead), .MEM_SIGN_L1(L1_mem_sign), .MEM_SIZE_L1(L1_mem_size), // L1 ports
                        .MEM_RESP_L1(L1_mem_resp), .MEM_RESP_VALID_L1(L1_mem_resp_valid),
                        
                        .MEM_ADDR2_L2(L2_mem_addr_2), .MEM_DOUT2_L2(L2_mem_data_in),  .MEM_READ2_L2(L2_memRead), .MEM_SIGN_L2(L2_mem_sign), .MEM_SIZE_L2(L2_mem_size), // L2 ports
                        .MEM_RESP_L2(L2_mem_resp), .MEM_RESP_VALID_L2(L2_mem_resp_valid),
                        
                        .MEM_ADDR2_S1(S1_mem_addr_2), .MEM_DIN2_S1(S1_mem_write_data), .MEM_WRITE_S1(S1_mem_write), .MEM_SIGN_S1(S1_mem_sign), .MEM_SIZE_S1(S1_mem_size), // S1 ports
                        .MEM_RESP_S1(s1_mem_resp), .MEM_RESP_VALID_S1(S1_mem_resp_valid),
                        
                        .MEM_ADDR2_S2(S2_mem_addr_2), .MEM_DIN2_S2(S2_mem_write_data), .MEM_WRITE_S2(S2_mem_write), .MEM_SIGN_S2(S2_mem_sign), .MEM_SIZE_S2(S2_mem_size), // S2 ports
                        .MEM_RESP_S2(S2_mem_resp), .MEM_RESP_VALID_S2(S2_mem_resp_valid)

                        );
    
    //============== ALU 1==============//    
    logic ALU1_done, ALU1_busy, ALU1_V1_valid, ALU1_V2_valid, ALU1_V3_valid;
    logic [31:0] ALU1_V1, ALU1_V2, ALU1_V3;
    RS_tag_type ALU1_rd_tag;
//    logic [31:0] ALU1_rs2_data;
    logic [2:0] ALU1_mem_type;
    logic [3:0] ALU1_alu_fun;
    ReservationStation #(ALU_1) RS_ALU1(.DISPATCH_TASK(DISPATCH_TASK), .dest_RS(dest_rs), .T1(T1), .T2(T2), .T3(T3),
                                        .cdb_in(cdb_data), .done(ALU1_done), .BUSY(ALU1_busy), .V1(ALU1_V1),
                                        .V2(ALU1_V2), .V3(ALU1_V3), .V1_valid(ALU1_V1_valid), .V2_valid(ALU1_V2_valid), .V3_valid(ALU1_V3_valid),
                                        .rd_tag(ALU1_rd_tag), .mem_type(ALU1_mem_type), .alu_fun(ALU1_alu_fun));
    
    logic [31:0] ALU1_CDB_val;
    RS_tag_type ALU1_CDB_tag;
    OTTER_ALU ALU1(.V1(ALU1_V1), .V2(ALU1_V2), .V1_valid(ALU1_V1_valid), .V2_valid(ALU1_V2_valid), .alu_fun(ALU1_alu_fun), .rd_tag(ALU1_rd_tag), 
                   .CDB_val(ALU1_CDB_val), .CDB_tag(ALU1_CDB_tag), .done(ALU1_done));
    
    
    //============== ALU 2==============//
    logic ALU2_done, ALU2_busy, ALU2_V1_valid, ALU2_V2_valid, ALU2_V3_valid;
    logic [31:0] ALU2_V1, ALU2_V2, ALU2_V3;
    RS_tag_type ALU2_rd_tag;
//    logic [31:0] ALU2_rs2_data;
    logic [2:0] ALU2_mem_type;
    logic [3:0] ALU2_alu_fun;
    ReservationStation #(ALU_2) RS_ALU2(.DISPATCH_TASK(DISPATCH_TASK), .dest_RS(dest_rs), .T1(T1), .T2(T2), .T3(T3),
                                        .cdb_in(cdb_data), .done(ALU2_done), .BUSY(ALU2_busy), .V1(ALU2_V1),
                                        .V2(ALU2_V2), .V3(ALU2_V3), .V1_valid(ALU2_V1_valid), .V2_valid(ALU2_V2_valid), .V3_valid(ALU2_V3_valid),
                                        .rd_tag(ALU2_rd_tag), .mem_type(ALU2_mem_type), .alu_fun(ALU2_alu_fun));
    
    logic [31:0] ALU2_CDB_val;
    RS_tag_type ALU2_CDB_tag;
    OTTER_ALU ALU2(.V1(ALU2_V1), 
                    .V2(ALU2_V2), 
                    .V1_valid(ALU2_V1_valid), 
                    .V2_valid(ALU2_V2_valid), 
                    .alu_fun(ALU2_alu_fun), 
                    .rd_tag(ALU2_rd_tag), 
                   .CDB_val(ALU2_CDB_val), 
                   .CDB_tag(ALU2_CDB_tag), 
                   .done(ALU2_done));
    
 
//    assign rs_busy = {S1_busy, S2_busy, L1_busy, L2_busy , ALU1_busy, ALU2_busy}; // concatenate each RS's busy bits
    assign rs_busy = {ALU2_busy, ALU1_busy, L2_busy, L1_busy, S2_busy, S1_busy};
    
    // Completion Queue
    CompletionQueue Completion(.CLK(CLK), .TAG_IN1(L1_CDB_tag), .DATA_IN1(L1_CDB_val), .TAG_IN2(L2_CDB_tag), .DATA_IN2(L2_CDB_val), 
                               .TAG_IN3(ALU1_CDB_tag), .DATA_IN3(ALU1_CDB_val), .TAG_IN4(ALU2_CDB_tag), .DATA_IN4(ALU2_CDB_val), .CDB_OUT(cdb_data));
    
//======================= END EXECUTE STAGE ===========================//



    
    

    // ************************ BEGIN PROGRAMMER ************************ 

//    assign mem_addr_after = s_prog_ram_we ? s_prog_ram_addr : MEM_aluResult;  // 2:1 mux
//    assign mem_data_after = s_prog_ram_we ? s_prog_ram_data : MEM_DIN2;  // 2:1 mux
//    assign mem_size_after = s_prog_ram_we ? 2'b10 : EX_MEM_instr.mem_type[1:0];  // 2:1 mux
//    assign mem_sign_after = s_prog_ram_we ? 1'b0 : EX_MEM_instr.mem_type[2];  // 2:1 mux
//    assign mem_we_after = s_prog_ram_we | EX_MEM_instr.memWrite;  // or gate
//    assign RESET = s_prog_mcu_reset | EXT_RESET;  // or gate

    // ************************ END PROGRAMMER ************************               
                           
    // ^ CHANGED aluResult to mem_addr_after FOR PROGRAMMER
    // ^ CHANGED B to mem_data_after FOR PROGRAMMER
    // ^ CHANGED memWrite to mem_we_after FOR PROGRAMMER
    // ^ CHANGED IR[13:12] to mem_size_after FOR PROGRAMMER
    // ^ CHANGED IR[14] to mem_sign_after FOR PROGRAMMER
     

    //MMIO /////////////////////////////////////////////////////           
    assign IOBUS_ADDR = mem_addr_after;  // CHANGED FROM aluResult TO mem_addr_after FOR PROGRAMMER
    assign IOBUS_OUT = mem_data_after;  // CHANGED FROM B TO mem_data_after FOR PROGRAMMER 
         
            
endmodule
