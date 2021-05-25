import cpu_types::*;

module Execute(
    input [31:0] EX_A, EX_B, MEM_aluResult, WB_aluResult, mem_data, EX_IR, EX_I_immed, EX_RS2,
    input instr_t DE_EX_instr, EX_MEM_instr, MEM_WB_instr,

    output [31:0] branch_pc, jump_pc, jalr_pc, int_pc, aluResult, // holds result of ALU operation
    output [2:0] pc_source,
    output jb_taken,
    output [31:0] MEM_RS2 // rs2 value to be outputted to the memory stage
);

    logic [2:0] func_3; // needed for checking branch conditions

    logic [1:0] forward_sel_A, forward_sel_B; // MUX control signals to choose forwarded data
    logic [31:0] aluA, aluB; // true inputs to the ALU module
      
    //pc target calculations
    assign jalr_pc = EX_I_immed + aluA;
    //assign branch_pc = pc + {{21{IR[31]}},IR[7],IR[30:25],IR[11:8] ,1'b0};   //word aligned addresses
    assign branch_pc = DE_EX_instr.pc + {{20{EX_IR[31]}},EX_IR[7],EX_IR[30:25],EX_IR[11:8],1'b0};   //byte aligned addresses
    assign jump_pc = DE_EX_instr.pc + {{12{EX_IR[31]}}, EX_IR[19:12], EX_IR[20],EX_IR[30:21],1'b0};
    assign int_pc = 0;
    
    logic br_taken,br_lt,br_eq,br_ltu; // contorl signals to handle jumps and branches
    logic jump_taken;
    
    logic [2:0] pc_src; // tmp values to make synthesis happy
    logic [31:0] mem_rs2;
    
    //Branch Condition Generator
    always_comb
    begin
        br_lt=0; br_eq=0; br_ltu=0;
        if($signed(aluA) < $signed(aluB)) br_lt=1;
        if(aluA==aluB) br_eq=1;
        if(aluA<aluB) br_ltu=1;
    end
    
    assign func_3 = EX_IR[14:12];
    
    always_comb // determine if a branch will be taken
    begin
        br_taken = 0;
        case(func_3)
            3'b000: br_taken = br_eq;     //BEQ 
            3'b001: br_taken = ~br_eq;    //BNE
            3'b100: br_taken = br_lt;     //BLT
            3'b101: br_taken = ~br_lt;    //BGE
            3'b110: br_taken = br_ltu;    //BLTU
            3'b111: br_taken = ~br_ltu;   //BGEU
            default: br_taken =0;
        endcase
        case(DE_EX_instr.opcode)    
            BRANCH: br_taken = br_taken;    // Make sure only Branches put up branch taken flag
            default: br_taken = 0;
        endcase
    end
    
    always_comb // PC_Source generation
    begin
        jump_taken = 0;
        case (DE_EX_instr.opcode) // use the opcode to determine which pc value to use
            JALR:   begin
                        pc_src = 3'b001;
                        jump_taken = 1;
                    end
            
            BRANCH: pc_src = (br_taken) ? 3'b010 : 3'b000;
            JAL:    begin 
                        pc_src = 3'b011;
                        jump_taken = 1;           // Not technically a branch but need to signal to nullify next instructions
                    end
            SYSTEM: pc_src = (func_3==3'b000)? 3'b101:3'b000; // func_3 = 3'b000 => mret
            default: pc_src = 3'b000;
        endcase
        if (DE_EX_instr.invalid) pc_src = 3'b000;    // If our instruction is invalid don't do target generation
    end
    
    // Generates select bits to choose between forwarded and non-forwarded data
    Forwarding_Unit FU(.RS1(DE_EX_instr.rs1_addr), .RS1_USED(DE_EX_instr.rs1_used), .RS2(DE_EX_instr.rs2_addr), 
                       .RS2_USED(DE_EX_instr.rs2_used), .EX_MEM_RD(EX_MEM_instr.rd_addr), .EX_MEM_REGWRITE(EX_MEM_instr.regWrite),
                       .MEM_WB_RD(MEM_WB_instr.rd_addr), .MEM_WB_REGWRITE(MEM_WB_instr.regWrite), .LD_HAZ(MEM_WB_instr.ld_haz), 
                       .OP(DE_EX_instr.opcode), .SEL_A(forward_sel_A), .SEL_B(forward_sel_B));
    
    // Adding two 3-1 MUXes here to handle data hazard forwarding
    always_comb
    begin
        case (forward_sel_A)
            2'b00: aluA = EX_A; // no forwarding was needed
            2'b01: aluA = MEM_aluResult; // forwarding was needed from ALU output going to memory stage
            2'b10: aluA = WB_aluResult; // fowarding was needed from ALU output going to writeback stage
            2'b11: aluA = mem_data; // forwarding was needed from data being loaded from memory module
            default: aluA = EX_A;
        endcase
    end
    
    always_comb
    begin
        case (forward_sel_B)
            2'b00: aluB = EX_B; // no forwarding was needed
            2'b01: aluB = MEM_aluResult; // forwarding was needed from ALU output going to memory stage
            2'b10: aluB = WB_aluResult; // fowarding was needed from ALU output going to writeback stage
            2'b11: aluB = mem_data; // forwarding was needed from data being loaded from memory module
            default: aluB = EX_B;
        endcase
    end
    
    // Creates a RISC-V ALU
    // Inputs are ALUCtl (the ALU control), ALU value inputs (ALUAin, ALUBin)
    // Outputs are ALUResultOut (the 64-bit output) and Zero (zero detection output)
    OTTER_ALU ALU (DE_EX_instr.alu_fun, aluA, aluB, aluResult); // the ALU
    
    always_comb
    begin // Handles possible store hazard with MEM_DIN2 data
        mem_rs2 = EX_RS2;
        if (DE_EX_instr.opcode == STORE)
            case (DE_EX_instr.rs2_addr)
                EX_MEM_instr.rd_addr: mem_rs2 = MEM_aluResult;
                MEM_WB_instr.rd_addr: mem_rs2 = WB_aluResult;
                default: mem_rs2 = EX_RS2;
            endcase
    end
    
    // Assign some outputs
    assign jb_taken = (br_taken || jump_taken) && (~DE_EX_instr.invalid); // Make sure we don't take branches if invalid instruction
    assign pc_source = pc_src;
    assign MEM_RS2 = mem_rs2;
    
    

endmodule