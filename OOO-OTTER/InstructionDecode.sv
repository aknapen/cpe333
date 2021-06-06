import cpu_types::*;

module InstructionDecode(
    input CLK,
    input [31:0] IF_ID_pc_0, IF_ID_pc_1,
    input [31:0] IR_0, IR_1,
    input regWrite,
    input regWrite_addr,
    input [31:0] WB_rfIn,
    output task_t TASK_0, TASK_1
    );

    

    //======================= BEGIN DECODE STAGE ===========================//
    
    task_t task_0, task_1; // translating instructions fetched to tasks sent to the issue queue
    
    logic opA_sel_0, opA_sel_1;// select bits for registers A and B MUXes
    logic [1:0] opB_sel_0, opB_sel_1; 
    
    logic [3:0] alu_fun_0, alu_fun_1; // specifies what operation the ALU will execute
    
    logic [1:0] wb_sel_0, wb_sel_1; // select bit for the writeback mux to the reg file
    
    logic intTaken = 0;
        
    // Instruction Decoder
    OTTER_CU_Decoder CU_DECODER(.CU_OPCODE_0(IR_0[6:0]), .CU_FUNC3_0(IR_0[14:12]), .CU_FUNC7_0(IR_0[31:25]),
                                .CU_OPCODE_1(IR_1[6:0]), .CU_FUNC3_1(IR_1[14:12]), .CU_FUNC7_1(IR_1[31:25]), 
                                .CU_ALU_SRCA_0(opA_sel_0), .CU_ALU_SRCB_0(opB_sel_0),.CU_ALU_FUN_0(alu_fun_0),
                                .CU_ALU_SRCA_1(opA_sel_1), .CU_ALU_SRCB_1(opB_sel_1),.CU_ALU_FUN_1(alu_fun_1),
                                .CU_RF_WR_SEL_0(wb_sel_0), .CU_RF_WR_SEL_1(wb_sel_1), .intTaken(intTaken));
    
    //======= MOVING REGISTER READS TO ISSUE TIME =========
//    logic [31:0] rs1_0, rs1_1, rs2_0, rs2_1;
    // Creates a RISC-V register file
//    OTTER_registerFile RF (IR_0[19:15], IR_0[24:20], IR_1[19:15], IR_1[24:20], regWrite_addr, WB_rfIn, regWrite, rs1_0, 
//                           rs1_1, rs2_0, rs2_1, CLK); // Register file
    
    logic [31:0] S_immed_0, S_immed_1, I_immed_0, I_immed_1, U_immed_0, U_immed_1;
    // Generate immediates
    assign S_immed_0 = {{20{IR_0[31]}},IR_0[31:25],IR_0[11:7]};
    assign I_immed_0 = {{20{IR_0[31]}},IR_0[31:20]};
    assign U_immed_0 = {IR_0[31:12],{12{1'b0}}};
    
    assign S_immed_1 = {{20{IR_1[31]}},IR_1[31:25],IR_1[11:7]};
    assign I_immed_1 = {{20{IR_1[31]}},IR_1[31:20]};
    assign U_immed_1 = {IR_1[31:12],{12{1'b0}}};
    
    
    //=========MOVING A AND B INPUT GENERATION TO ISSUE TIME============
//    logic [31:0] A_0, A_1, B_0, B_1; // ALU  A and B that will be sent to the Execute Stage
//    // Creates a 2-to-1 multiplexor used to select the A input of the ALU 
//    Mult2to1 ALUAinput0 (rs1_0, U_immed_0, opA_sel_0, A_0);
//    Mult2to1 ALUAinput1 (rs1_1, U_immed_1, opA_sel_1, A_1);
    
//    // Creates a 4-to-1 multiplexor used to select the B input of the ALU
//    Mult4to1 ALUBinput0 (rs2_0, I_immed_0, S_immed_0, IF_ID_pc_0, opB_sel_0, B_0);
//    Mult4to1 ALUBinput1 (rs2_1, I_immed_1, S_immed_1, IF_ID_pc_1, opB_sel_1, B_1);
    
    logic rs1_used_0, rs1_used_1, rs2_used, rd_used, memWrite, memRead2, regWrite;  // for calculating struct fields
    
    opcode_t opcode_0, opcode_1;
    assign opcode_0 = opcode_t'(IR_0[6:0]);
    assign opcode_1 = opcode_t'(IR_1[6:0]);
    
    // Task Generation
    always_comb // generate task 0
    begin
//        task_0.A = A_0; // Moving A and B generation to issue time
//        task_0.B = B_0;
        task_0.opcode = opcode_0;
        task_0.rd_addr = IR_0[11:7];
        task_0.rd_used = ((opcode_0 != BRANCH) && // only BRANCH and STORE instructions don't use an rd
                          (opcode_0 != NOP) &&
                          (opcode_0 != STORE)) ? 1 : 0;
        task_0.rs1_addr = IR_0[19:15];
        task_0.rs1_used = ((opcode_0 != LUI) && // only LUI, AUIPC, and JAL instruction don't use rs1
                           (opcode_0 != AUIPC) &&
                           (opcode_0 != NOP) &&
                           (opcode_0 != JAL)) ? 1 : 0;
        task_0.rs2_addr = IR_0[24:20];
//        task_0.rs2_data = rs2_0; // Moving rs2 fetch to issue time
        task_0.rs2_used = ((opcode_0 == BRANCH) || // only BRANCH, STORE, and OP instruction use rs2
                           (opcode_0 == STORE) ||
                           (opcode_0 == OP)) ? 1 : 0;
        task_0.alu_fun = alu_fun_0;
        task_0.mem_type = IR_0[14:12];
        task_0.wb_sel = wb_sel_0;
        task_0.opA_sel = opA_sel_0;
        task_0.U_immed = U_immed_0;
        task_0.opB_sel = opB_sel_0;
        task_0.I_immed = I_immed_0;
        task_0.S_immed = S_immed_0;
        task_0.pc =  IF_ID_pc_0;
    end
    
    always_comb // generate task 1
    begin
//        task_1.A = A_1; // Moving A and B generation to issue time
//        task_1.B = B_1;
        task_1.opcode = opcode_1;
        task_1.rd_addr = IR_1[11:7];
        task_1.rd_used = ((opcode_1 != BRANCH) && // only BRANCH and STORE instructions don't use an rd
                          (opcode_1 != NOP) &&
                          (opcode_1 != STORE)) ? 1 : 0;
        task_1.rs1_addr = IR_1[19:15];
        task_1.rs1_used = ((opcode_1 != LUI) && // only LUI, AUIPC, and JAL instruction don't use rs1
                           (opcode_1 != AUIPC) &&
                           (opcode_1 != NOP) &&
                           (opcode_1 != JAL)) ? 1 : 0;
        task_1.rs2_addr = IR_1[24:20];
//        task_1.rs2_data = rs2_1; // Moving rs2 fetch to issue time
        task_1.rs2_used = ((opcode_1 == BRANCH) || // only BRANCH, STORE, and OP instruction use rs2
                           (opcode_1 == STORE) ||
                           (opcode_1 == OP)) ? 1 : 0;
        task_1.alu_fun = alu_fun_1;
        task_1.mem_type = IR_1[14:12];
        task_1.wb_sel = wb_sel_1;
        task_1.opA_sel = opA_sel_1;
        task_1.U_immed = U_immed_1;
        task_1.opB_sel = opB_sel_1;
        task_1.I_immed = I_immed_1;
        task_1.S_immed = S_immed_1;
        task_1.pc =  IF_ID_pc_1;
    end
    
    assign TASK_0 = task_0;
    assign TASK_1 = task_1;
                                                            
//======================= END DECODE STAGE ===========================//
    
endmodule