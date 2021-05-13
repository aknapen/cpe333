import cpu_types::*;

module InstructionDecode(
    input CLK,

    input [31:0] IF_ID_pc,
    input [31:0] IR,
    input instr_t DE_EX_instr, // Instruction currently in the execute stage
    input instr_t EX_MEM_instr, // Instruction currently in the mem stage
    input instr_t MEM_WB_instr, // Instruction currently in the WB stage

    input jb_taken,
    input [31:0] WB_rfIn,

    output [31:0] DE_A, DE_B, // ALU  A and B that will be sent to the Execute Stage
    output [31:0] rs2, // Need this later for mem
    output[31:0] I_immed,
    output ld_haz,
    output instr_t instr
    );

    

    //======================= BEGIN DECODE STAGE ===========================//
    
    logic opA_sel;// select bits for registers A and B MUXes
    logic [1:0] opB_sel; 
    
    logic [3:0] alu_fun; // specifies what operation the ALU will execute
    logic [31:0] rs1, S_immed, U_immed; // ,aluBin,aluAin; // functional signals for the decode stage
    
    logic rs1_used, rs2_used, rd_used, memWrite, memRead2, regWrite;  // for calculating struct fields
    
    logic [1:0] wb_sel; // select bit for the writeback mux to the reg file
    
    logic intTaken = 0;

    opcode_t opcode; // specifies the opcode of the instruciton in the decode stage
        
    // Load-Use Hazard Detection
    assign ld_haz = (!jb_taken && !DE_EX_instr.br_taken && !DE_EX_instr.invalid &&
                    (DE_EX_instr.opcode == LOAD) && ((DE_EX_instr.rd_addr == IR[19:15]) || (DE_EX_instr.rd_addr == IR[24:20]))) ? 1 : 0;
//    always_comb
//    begin
//        ld_haz = 0;
//        if (!jb_taken && !DE_EX_instr.br_taken && !DE_EX_instr.invalid &&
//            (DE_EX_instr.opcode == LOAD) && ((DE_EX_instr.rd_addr == IR[19:15]) || (DE_EX_instr.rd_addr == IR[24:20]))) 
//        begin
//            ld_haz = 1;
//        end
//    end
    
    // Creates a RISC-V register file
    OTTER_registerFile RF (IR[19:15], IR[24:20], MEM_WB_instr.rd_addr, WB_rfIn, MEM_WB_instr.regWrite, rs1, rs2, CLK); // Register file
    
    // Instruction Decoder
    OTTER_CU_Decoder CU_DECODER(.CU_OPCODE(IR[6:0]), .CU_FUNC3(IR[14:12]),.CU_FUNC7(IR[31:25]), 
             .CU_ALU_SRCA(opA_sel), .CU_ALU_SRCB(opB_sel),.CU_ALU_FUN(alu_fun),.CU_RF_WR_SEL(wb_sel),
             .intTaken(intTaken));    
    
    // Assign all control signals to be pushed through the pipeline
    assign opcode = opcode_t'(IR[6:0]); 
    assign rs1_used = ((opcode != LUI) && // only LUI, AUIPC, and JAL instruction don't use rs1
                       (opcode != AUIPC) &&
                       (opcode != NOP) &&
                       (opcode != JAL)) ? 1 : 0;
    assign rs2_used = ((opcode == BRANCH) || // only BRANCH, STORE, and OP instruction use rs2
                       (opcode == STORE) ||
                       (opcode == OP)) ? 1 : 0;
    assign rd_used = ((opcode != BRANCH) && // only BRANCH and STORE instructions don't use an rd
                      (opcode != NOP) &&
                      (opcode != STORE)) ? 1 : 0;  
    assign memWrite = (opcode == STORE) ? 1 : 0; // only enable mem write on a store instruction
    assign memRead2 = ((opcode == LOAD)) ? 1 : 0; // only enable read from mem on a load instruction
    assign regWrite = ((opcode != BRANCH) && // no rd for BRANCH or STORE instructions
                       (opcode != NOP) &&
                       (opcode != STORE)) ? 1 : 0;       
                                                                           
    // Generate immediates
    assign S_immed = {{20{IR[31]}},IR[31:25],IR[11:7]};
    assign I_immed = {{20{IR[31]}},IR[31:20]};
    assign U_immed = {IR[31:12],{12{1'b0}}};
    
    // Creates a 2-to-1 multiplexor used to select the A input of the ALU 
    Mult2to1 ALUAinput (rs1, U_immed, opA_sel, DE_A);
    // assign DE_A = aluAin; // ALU input A to transfer to the execute stage
    
    // Creates a 4-to-1 multiplexor used to select the B input of the ALU
    Mult4to1 ALUBinput (rs2, I_immed, S_immed, IF_ID_pc, opB_sel, DE_B);
    // assign DE_B = aluBin; // ALU input B to transfer to the execute stage

//======================= END DECODE STAGE ===========================//
    
    always_comb
    begin
        instr = 72'b0;
        if (~ld_haz) // Need to prevent passing instruction from decode to execute on a load-use hazard
        begin // otherwise, assign decode control signals to be pushed to execute stage
            instr.opcode = opcode;
            instr.invalid = (jb_taken) || (EX_MEM_instr.br_taken); // Instruction invalid if either of prev2 instructions took a branch
            instr.rs1_addr = IR[19:15];
            instr.rs2_addr = IR[24:20];
            instr.rd_addr = IR[11:7];
            instr.rs1_used = rs1_used;
            instr.rs2_used = rs2_used;
            instr.rd_used = rd_used;
            instr.alu_fun = alu_fun;
            instr.memWrite = (memWrite) && (~instr.invalid); // If instruction is invalid don't write to mem
            instr.memRead2 = memRead2;
            instr.regWrite = (regWrite) && (~instr.invalid);  // If instruction is invalid don't write to reg
            instr.rf_wr_sel = wb_sel;
            instr.mem_type = IR[14:12]; // holds size and sign for memory module (funct3 in instruction)
            instr.ld_haz = ld_haz;
            instr.pc = IF_ID_pc; // get pc value from fetch stage
            instr.br_taken = 0;   
        end
    end

endmodule