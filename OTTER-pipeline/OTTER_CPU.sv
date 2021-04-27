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
        
    // ************************ BEGIN PROGRAMMER ************************ 

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
    instr_t NO_OP = 69'b0;
    
    logic [31:0] IF_ID_pc;
      
    wire mepcWrite, csrWrite,intCLR, mie, intTaken;
    wire [31:0] mepc, mtvec;
   
//=======================  BEGIN FETCH STAGE ===========================//
    
    logic [31:0] pc_in, pc_out, next_pc, jalr_pc, branch_pc, jump_pc, int_pc;   
    logic pcWrite;
    logic [2:0] pc_source;
    logic memRead1;
        
    // Creates a 6-to-1 multiplexor used to select the source of the next PC
    Mult6to1 PCdatasrc (next_pc, jalr_pc, branch_pc, jump_pc, mtvec, mepc, pc_source, pc_in);
       
    assign pcWrite = ~ld_haz; // only allow new PC through if we're not stalling
     
    //PC is byte-addressed but our memory is word addressed 
    ProgCount PC (.PC_CLK(CLK), .PC_RST(RESET), .PC_LD(pcWrite),
                  .PC_DIN(pc_in), .PC_COUNT(pc_out));   
                 
    assign next_pc = pc_out + 4;    //PC is byte aligned, memory is word aligned (NEED TO CALC next_pc HERE SO IT'S READY BY NEXT INSTRUCTION                          
    assign memRead1 = 1; // can hardcode this to 1 since we always want to read an instr in the fetch stage
   
    //=======================  END FETCH STAGE ===========================//
    
    logic [31:0] fetch_pc;
    
    always_comb
    begin
        fetch_pc = pc_out;
        if (ld_haz) fetch_pc = 32'b0;
    end
    
    // PC REGISTER
    always_ff @(posedge CLK) // transfers PC from fetch to decode
    begin
        IF_ID_pc <= fetch_pc; // delay PC from fetch stage
    end
    
   //======================= BEGIN DECODE STAGE ===========================//

    logic [31:0] IR; // instruction from fetch stage
    logic [31:0] DE_A, DE_B; // inputs A and B that will be sent to the Execute Stage
    
    logic opA_sel;// select bits for registers A and B MUXes
    logic [1:0] opB_sel; 
    
    logic [3:0] alu_fun;
    logic [31:0] rs1, rs2, I_immed,S_immed,U_immed,aluBin,aluAin;
    
    logic rs1_used, rs2_used, rd_used, memWrite, memRead2, regWrite;  // for calculating struct fields
    
    logic [1:0] wb_sel;
    
    logic ld_haz; // used to detect a load-use hazard
    opcode_t opcode;
    
    // Load-Use Hazard Detection
    always_comb
    begin
        ld_haz = 0;
        if ((opcode == LOAD) && ((DE_EX_instr.rd_addr == IR[19:15])  || (DE_EX_instr.rd_addr == IR[24:20]))) ld_haz = 1;
    end
    
    // Creates a RISC-V register file
    OTTER_registerFile RF (IR[19:15], IR[24:20], MEM_WB_instr.rd_addr, WB_rfIn, MEM_WB_instr.regWrite, rs1, rs2, CLK); // Register file
    
    // Instruction Decoder
    OTTER_CU_Decoder CU_DECODER(.CU_OPCODE(IR[6:0]), .CU_FUNC3(IR[14:12]),.CU_FUNC7(IR[31:25]), 
             .CU_ALU_SRCA(opA_sel), .CU_ALU_SRCB(opB_sel),.CU_ALU_FUN(alu_fun),.CU_RF_WR_SEL(wb_sel),
             .intTaken(intTaken));    
    
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
    Mult2to1 ALUAinput (rs1, U_immed, opA_sel, aluAin);
    assign DE_A = aluAin;
    
    // Creates a 4-to-1 multiplexor used to select the B input of the ALU
    Mult4to1 ALUBinput (rs2, I_immed, S_immed, IF_ID_pc, opB_sel, aluBin);
    assign DE_B = aluBin;

//======================= END DECODE STAGE ===========================//

    logic [31:0] EX_A, EX_B, EX_IR, EX_RS2, EX_I_immed;

    always_ff @(posedge CLK) // to push ALU inputs and instruction from Decode to Execute stage
    begin // DO THESE NEED TO BE ZEROED OUT ON LOAD-USE HAZARD??? Don't think so b/c all control signals in instr_t should be 0
        EX_A <= DE_A;
        EX_B <= DE_B;
        EX_IR <= IR;
        EX_RS2 <= rs2; // Need this later for mem
        EX_I_immed <= I_immed;
    end
    
    instr_t instr;
    always_comb
    begin
        case (ld_haz)
            0: 
            begin
                instr.opcode <= opcode;
    //            instr.invalid <= EX_MEM_instr.br_taken || MEM_WB_instr.br_taken; // Instruction invalid if either of prev2 instructions took a branch
                instr.invalid = (jb_taken) || (EX_MEM_instr.br_taken); // Instruction invalid if either of prev2 instructions took a branch
                instr.rs1_addr <= IR[19:15];
                instr.rs2_addr <= IR[24:20];
                instr.rd_addr <= IR[11:7];
                instr.rs1_used <= rs1_used;
                instr.rs2_used <= rs2_used;
                instr.rd_used <= rd_used;
                instr.alu_fun <= alu_fun;
                instr.memWrite = (memWrite) && (~instr.invalid); // If instruction is invalid don't write to mem
                instr.memRead2 <= memRead2;
                instr.regWrite = (regWrite) && (~instr.invalid);  // If instruction is invalid don't write to reg
                instr.rf_wr_sel <= wb_sel;
                instr.mem_type <= IR[14:12]; // holds size and sign for memory module (funct3 in instruction)
                instr.ld_haz <= ld_haz;
                instr.pc <= IF_ID_pc; // get pc value from fetch stage
                instr.br_taken <= 0;   
            end
            
            1: instr = 72'b0;
        endcase
    end

    always_ff @(posedge CLK)
    begin
        DE_EX_instr <= instr;
    end
      
//======================= BEGIN EXECUTE STAGE ===========================//
    logic [2:0] func_3; // needed for checking branch conditions
    logic [31:0] aluResult;
    
    //pc target calculations
    assign jalr_pc = EX_I_immed + aluA;
    //assign branch_pc = pc + {{21{IR[31]}},IR[7],IR[30:25],IR[11:8] ,1'b0};   //word aligned addresses
    assign branch_pc = DE_EX_instr.pc + {{20{EX_IR[31]}},EX_IR[7],EX_IR[30:25],EX_IR[11:8],1'b0};   //byte aligned addresses
    assign jump_pc = DE_EX_instr.pc + {{12{EX_IR[31]}}, EX_IR[19:12], EX_IR[20],EX_IR[30:21],1'b0};
    assign int_pc = 0;
    
    logic br_taken,br_lt,br_eq,br_ltu;
    logic jump_taken, jb_taken;
    
    logic [1:0] forward_sel_A, forward_sel_B; // MUX control signals to choose forwarded data
    logic [31:0] aluA, aluB;
    
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
//        EX_MEM_instr.br_taken <= br_taken;             // Used to identify branches
    end
    
    always_comb // PC_Source generation
    begin
        jump_taken = 0;
        case (DE_EX_instr.opcode)
            JALR:   begin
                        pc_source = 3'b001;
                        jump_taken = 1;
                    end
            
            BRANCH: pc_source = (br_taken) ? 3'b010 : 3'b000;
            JAL:    begin 
                        pc_source = 3'b011;
                        jump_taken = 1;           // Not technically a branch but need to signal to nullify next instructions
                    end
            SYSTEM: pc_source = (func_3==3'b000)? 3'b101:3'b000; // func_3 = 3'b000 => mret
            default: pc_source = 3'b000;
        endcase
        if (DE_EX_instr.invalid) pc_source = 3'b000;    // If our instruction is invalid don't do target generation
    end
    
    // Generates select bits to choose between forwarded and non-forwarded data
    Forwarding_Unit FU(.RS1(DE_EX_instr.rs1_addr), .RS1_USED(DE_EX_instr.rs1_used), .RS2(DE_EX_instr.rs2_addr), 
                       .RS2_USED(DE_EX_instr.rs2_used), .EX_MEM_RD(EX_MEM_instr.rd_addr), .EX_MEM_REGWRITE(EX_MEM_instr.regWrite),
                       .MEM_WB_RD(MEM_WB_instr.rd_addr), .MEM_WB_REGWRITE(MEM_WB_instr.regWrite), .LD_HAZ(MEM_WB_instr.ld_haz), 
                       .OP(DE_EX_instr.opcode), .SEL_A(forward_sel_A), .SEL_B(forward_sel_B));
    
    // Adding two 3-1 MUXes here to handle data hazards
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
    

//======================= END EXECUTE STAGE ===========================//
    logic [31:0] MEM_aluResult;
    logic [31:0] MEM_RS2, MEM_DIN2, MEM_I_immed;

    always_comb
    begin
        jb_taken = 0;
        jb_taken <= (br_taken || jump_taken) && (~DE_EX_instr.invalid); // Make sure we don't take branches if invalid instruction
    end
    
    always_comb
    begin // Handles possible store hazard with MEM_DIN2 data
        if (DE_EX_instr.opcode == STORE)
            case (DE_EX_instr.rs2_addr)
                EX_MEM_instr.rd_addr: MEM_RS2 = MEM_aluResult;
                MEM_WB_instr.rd_addr: MEM_RS2 = WB_aluResult;
                default: MEM_RS2 = EX_RS2;
            endcase
    end
    
    always_ff @(posedge CLK) // to push intsr_t through the pipeline stages
    begin
        EX_MEM_instr <= DE_EX_instr;
        EX_MEM_instr.br_taken <= jb_taken;
        MEM_aluResult <= aluResult;   
        MEM_DIN2 <= MEM_RS2; // Need this for din2 into memory
        MEM_I_immed <= EX_I_immed; // Need this for CSR
    end
    
//======================= BEGIN MEMORY STAGE ===========================//
    logic [31:0] mem_data;
    
    // Sets up memory for the fetch stage and for memory accessing in Memory stage
    // In the future need to check on IO and Programmer stuff
    OTTER_mem_byte #(14) memory  (.MEM_CLK(CLK),.MEM_ADDR1(pc_out),.MEM_ADDR2(MEM_aluResult),.MEM_DIN2(MEM_DIN2),
                               .MEM_WRITE2(EX_MEM_instr.memWrite),.MEM_READ1(memRead1),.MEM_READ2(EX_MEM_instr.memRead2), .LD_HAZ(ld_haz),
                               .ERR(),.MEM_DOUT1(IR),.MEM_DOUT2(mem_data),.IO_IN(IOBUS_IN),.IO_WR(IOBUS_WR),.MEM_SIZE(EX_MEM_instr.mem_type[1:0]),.MEM_SIGN(mem_sign_after));
    
//======================= END MEMORY STAGE ===========================//

    logic [31:0] WB_aluResult, WB_I_immed;
    
    always_ff @(posedge CLK) // to push intsr_t through the pipeline stages and result of the memory
    begin
        MEM_WB_instr <= EX_MEM_instr;
        WB_aluResult <= MEM_aluResult;
        WB_I_immed <= MEM_I_immed;
    end
                               
//======================= BEGIN WRITEBACK STAGE ===========================//

    logic [31:0] csr_reg;
    
    // Technically, the writeback stage uses the register file since it writes back to the registers 
   
    CSR CSRs(.clk(CLK),.rst(RESET),.intTaken(intTaken),.addr(WB_I_immed),.next_pc(MEM_WB_instr.pc),.wd(WB_aluResult),.wr_en(csrWrite),
           .rd(csr_reg),.mepc(mepc),.mtvec(mtvec),.mie(mie));  
    
    //Creates 4-to-1 multiplexor used to select reg write back data
    // Mult4 to 1 ( PC+4, CSR reg, dout2 from mem, alu result, sel = wb_sel from decoder, out = finished register in
    
    // WB_rfIn is connected to the reg file 
    logic [31:0] MEM_PC;
    assign MEM_PC = MEM_WB_instr.pc + 4;
    logic [31:0] WB_rfIn; 
    Mult4to1 regWriteback (MEM_PC,csr_reg, mem_data, WB_aluResult, MEM_WB_instr.rf_wr_sel, WB_rfIn);
    

    // ************************ BEGIN PROGRAMMER ************************ 

    assign mem_addr_after = s_prog_ram_we ? s_prog_ram_addr : MEM_aluResult;  // 2:1 mux
    assign mem_data_after = s_prog_ram_we ? s_prog_ram_data : MEM_RS2;  // 2:1 mux
    assign mem_size_after = s_prog_ram_we ? 2'b10 : EX_MEM_instr.mem_type[1:0];  // 2:1 mux
    assign mem_sign_after = s_prog_ram_we ? 1'b0 : EX_MEM_instr.mem_type[2];  // 2:1 mux
    assign mem_we_after = s_prog_ram_we | EX_MEM_instr.memWrite;  // or gate
    assign RESET = s_prog_mcu_reset | EXT_RESET;  // or gate

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
