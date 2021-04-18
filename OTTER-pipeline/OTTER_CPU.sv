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
        SYSTEM   = 7'b1110011
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
        logic [31:0] pc;
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
    
    logic [31:0] IF_ID_pc;
    
    wire [31:0] WB_rfIN,csr_reg, mem_data;
    
    wire [1:0] wb_sel;
        
    wire mepcWrite, csrWrite,intCLR, mie, intTaken;
    wire [31:0] mepc, mtvec;
   
//=======================  BEGIN FETCH STAGE ===========================//
    
    logic [31:0] pc_in, pc_out, next_pc, jalr_pc, branch_pc, jump_pc, int_pc;   
    logic pcWrite;
    logic [2:0] pc_source;
    logic memRead1;
    
    // Creates a 2-to-1 multiplexor used to select the source of the next PC
    Mult6to1 PCdatasrc (next_pc, jalr_pc, branch_pc, jump_pc, mtvec, mepc, pc_source, pc_in); // TODO: need to clk pc_sel from Decode stage???
    
    assign pcWrite = 1; // since there will be no hazards, can write new PC value every CLK cycle
     
    //PC is byte-addressed but our memory is word addressed 
    ProgCount PC (.PC_CLK(CLK), .PC_RST(RESET), .PC_LD(pcWrite),
                 .PC_DIN(pc_in), .PC_COUNT(pc_out));   
                 
    assign next_pc = pc_out + 4;    //PC is byte aligned, memory is word aligned (NEED TO CALC next_pc HERE SO IT'S READY BY NEXT INSTRUCTION                          
    assign memRead1 = 1; // can hardcode this to 1 since we always want to read an instr in the fetch stage
   
    //=======================  END FETCH STAGE ===========================//
    
    // PC REGISTER
    always_ff @(posedge CLK) // transfers PC from fetch to decode
    begin
        IF_ID_pc <= pc_out; // delay PC from fetch stage
    end
    
   //======================= BEGIN DECODE STAGE ===========================//

    logic [31:0] IR; // instruction from fetch stage
    logic [31:0] DE_A, DE_B; // inputs A and B that will be sent to the Execute Stage
    
    logic opA_sel;// select bits for registers A and B MUXes
    logic [1:0] opB_sel; 
    
    logic [3:0] alu_fun;
    logic [31:0] rs1, rs2, I_immed,S_immed,U_immed,aluBin,aluAin;
    
    logic rs1_used, rs2_used, rd_used, memWrite, memRead2, regWrite;  // for calculating struct fields
    opcode_t opcode;
    
    // Creates a RISC-V register file
    OTTER_registerFile RF (IR[19:15], IR[24:20], MEM_WB_instr.rd_addr, WB_rfIn, MEM_WB_instr.regWrite, rs1, rs2, CLK); // Register file
    
    // Instruction Decoder
    OTTER_CU_Decoder CU_DECODER(.CU_OPCODE(IR[6:0]), .CU_FUNC3(IR[14:12]),.CU_FUNC7(IR[31:25]), 
             .CU_ALU_SRCA(opA_sel), .CU_ALU_SRCB(opB_sel),.CU_ALU_FUN(alu_fun),.CU_RF_WR_SEL(wb_sel),
             .intTaken(intTaken));    
             
    // Take care of logic need to assign values pushed through pipeline in the struct
    assign opcode = opcode_t'(IR[6:0]);
    assign rs1_used = ((opcode != LUI) && // only LUI, AUIPC, and JAL instruction don't use rs1
                       (opcode != AUIPC) &&
                       (opcode != JAL)) ? 1 : 0;
    assign rs2_used = ((opcode == BRANCH) || // only BRANCH, STORE, and OP instruction use rs2
                       (opcode == STORE) ||
                       (opcode == OP)) ? 1 : 0;
    assign rd_used = ((opcode != BRANCH) && // only BRANCH and STORE instructions don't use an rd
                      (opcode != STORE)) ? 1 : 0;  
    assign memWrite = (opcode == STORE) ? 1 : 0; // only enable mem write on a store instruction
    assign memRead2 = (opcode == LOAD) ? 1 : 0; // only enable read from mem on a load instruction
    assign regWrite = ((opcode != BRANCH) && // no rd for BRANCH or STORE instructions
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

    logic [31:0] EX_A, EX_B, EX_IR, EX_RS2;
    always_ff @(posedge CLK) // to push ALU inputs and instruction from Decode to Execute stage
    begin
        EX_A <= DE_A;
        EX_B <= DE_B;
        EX_IR <= IR;
        EX_RS2 <= rs2; // Need this later for mem
    end
    
    always_ff @(posedge CLK) // to push struct info from Decode to Execute Stage
    begin
        DE_EX_instr.opcode <= opcode;
        DE_EX_instr.rs1_addr <= IR[19:15];
        DE_EX_instr.rs2_addr <= IR[24:20];
        DE_EX_instr.rd_addr <= IR[11:7];
        DE_EX_instr.rs1_used <= rs1_used;
        DE_EX_instr.rs2_used <= rs2_used;
        DE_EX_instr.rd_used <= rd_used;
        DE_EX_instr.alu_fun <= alu_fun;
        DE_EX_instr.memWrite <= memWrite;
        DE_EX_instr.memRead2 <= memRead2;
        DE_EX_instr.regWrite <= regWrite;  
        DE_EX_instr.rf_wr_sel <= wb_sel;
        DE_EX_instr.mem_type <= IR[14:12]; // holds size and sign for memory module (funct3 in instruction)
        DE_EX_instr.pc <= IF_ID_pc; // get pc value from fetch stage                                                                        
    end  
      
//======================= BEGIN EXECUTE STAGE ===========================//
    logic [2:0] func_3; // needed for checking branch conditions
    logic [31:0] aluResult;
    
    //pc target calculations
    assign jalr_pc = I_immed + EX_A;
    //assign branch_pc = pc + {{21{IR[31]}},IR[7],IR[30:25],IR[11:8] ,1'b0};   //word aligned addresses
    assign branch_pc = DE_EX_instr.pc + {{20{IR[31]}},EX_IR[7],EX_IR[30:25],EX_IR[11:8],1'b0};   //byte aligned addresses
    assign jump_pc = DE_EX_instr.pc + {{12{EX_IR[31]}}, EX_IR[19:12], EX_IR[20],EX_IR[30:21],1'b0};
    assign int_pc = 0;
    
    logic br_taken,br_lt,br_eq,br_ltu;
    
    //Branch Condition Generator
    always_comb
    begin
        br_lt=0; br_eq=0; br_ltu=0;
        if($signed(EX_A) < $signed(EX_B)) br_lt=1;
        if(EX_A==EX_B) br_eq=1;
        if(EX_A<EX_B) br_ltu=1;
    end
    
    assign func_3 = EX_IR[14:12];
    
    always_comb // determine if a branch will be taken
    begin
        case(func_3)
            3'b000: br_taken = br_eq;     //BEQ 
            3'b001: br_taken = ~br_eq;    //BNE
            3'b100: br_taken = br_lt;     //BLT
            3'b101: br_taken = ~br_lt;    //BGE
            3'b110: br_taken = br_ltu;    //BLTU
            3'b111: br_taken = ~br_ltu;   //BGEU
            default: br_taken =0;
        endcase
    end
    
    always_comb // PC_Source generation
    begin
        case (DE_EX_instr.opcode)
            JALR: pc_source = 3'b000;
            BRANCH: pc_source = (br_taken) ? 3'b001 : 3'b000;
            JAL: pc_source = 3'b010;
            SYSTEM: pc_source = (func_3==3'b000)? 3'b101:3'b000; // func_3 = 3'b000 => mret
            default: pc_source = 3'b000;
        endcase
    end
    
    // Creates a RISC-V ALU
    // Inputs are ALUCtl (the ALU control), ALU value inputs (ALUAin, ALUBin)
    // Outputs are ALUResultOut (the 64-bit output) and Zero (zero detection output)
    OTTER_ALU ALU (DE_EX_instr.alu_fun, EX_A, EX_B, aluResult); // the ALU
    

//======================= END EXECUTE STAGE ===========================//
    logic [31:0] MEM_aluResult;
    logic [31:0] MEM_RS2;
    
    always_ff @(posedge CLK) // to push intsr_t through the pipeline stages
    begin
        EX_MEM_instr <= DE_EX_instr;
        MEM_aluResult <= aluResult;   
        MEM_RS2 <= EX_RS2; // Need this for din2 into memory
    end
    
//======================= BEGIN MEMORY STAGE ===========================//
    
    // Original Memory 
//    OTTER_mem_byte #(14) memory  (.MEM_CLK(CLK),.MEM_ADDR1(pc_out),.MEM_ADDR2(mem_addr_after),.MEM_DIN2(mem_data_after),
//                               .MEM_WRITE2(mem_we_after),.MEM_READ1(memRead1),.MEM_READ2(memRead2),
//                               .ERR(),.MEM_DOUT1(IR),.MEM_DOUT2(mem_data),.IO_IN(IOBUS_IN),.IO_WR(IOBUS_WR),.MEM_SIZE(mem_size_after),.MEM_SIGN(mem_sign_after));
    
    //    assign mem_data_after = s_prog_ram_we ? s_prog_ram_data : MEM_RS2;  // I think something needs to be done with this but not sure how it works
//        assign mem_size_after = s_prog_ram_we ? 2'b10 : IR[13:12];  // 2:1 mux

    // Sets up memory for the fetch stage and for memory accessing in Memory stage
    // In the future need to check on IO and Programmer stuff
    OTTER_mem_byte #(14) memory  (.MEM_CLK(CLK),.MEM_ADDR1(pc_out),.MEM_ADDR2(MEM_aluResult),.MEM_DIN2(MEM_RS2),
                               .MEM_WRITE2(EX_MEM_instr.memWrite),.MEM_READ1(memRead),.MEM_READ2(EX_MEM_instr.memRead2),
                               .ERR(),.MEM_DOUT1(IR),.MEM_DOUT2(mem_data),.IO_IN(IOBUS_IN),.IO_WR(IOBUS_WR),.MEM_SIZE(EX_MEM_instr.mem_type),.MEM_SIGN(mem_sign_after));

//======================= END MEMORY STAGE ===========================//
    logic [31:0] MEM_WB_out;
    logic [31:0] WB_aluResult;
    
    always_ff @(posedge CLK) // to push intsr_t through the pipeline stages and result of the memory
    begin
        MEM_WB_instr <= EX_MEM_instr;
        MEM_WB_out <= mem_data;
        WB_aluResult <= MEM_aluResult;
    end
                               
//======================= BEGIN WRITEBACK STAGE ===========================//

// Technically, the writeback stage uses the register file since it writes back to the registers 
    
    //CSR registers and interrupt logic
    CSR CSRs(.clk(CLK),.rst(RESET),.intTaken(intTaken),.addr(IR[31:20]),.next_pc(pc),.wd(aluResult),.wr_en(csrWrite),
           .rd(csr_reg),.mepc(mepc),.mtvec(mtvec),.mie(mie));  
    
    //Creates 4-to-1 multiplexor used to select reg write back data
    // Mult4 to 1 ( PC+4, CSR reg, dout2 from mem, alu result, sel = wb_sel from decoder, out = finished register in
    
    // WB_rfIn is connected to the reg file 
    Mult4to1 regWriteback (MEM_WB_instr.pc + 4,csr_reg,MEM_WB_out, WB_aluResult, MEM_WB_instr.rf_wr_sel, WB_rfIn);
    

    // ************************ BEGIN PROGRAMMER ************************ 

    assign mem_addr_after = s_prog_ram_we ? s_prog_ram_addr : aluResult;  // 2:1 mux
    assign mem_data_after = s_prog_ram_we ? s_prog_ram_data : rs2;  // 2:1 mux
    assign mem_size_after = s_prog_ram_we ? 2'b10 : IR[13:12];  // 2:1 mux
    assign mem_sign_after = s_prog_ram_we ? 1'b0 : IR[14];  // 2:1 mux
    assign mem_we_after = s_prog_ram_we | memWrite;  // or gate
    assign RESET = s_prog_mcu_reset | EXT_RESET;  // or gate

    // ************************ END PROGRAMMER ************************               
                           
    // ^ CHANGED aluResult to mem_addr_after FOR PROGRAMMER
    // ^ CHANGED B to mem_data_after FOR PROGRAMMER
    // ^ CHANGED memWrite to mem_we_after FOR PROGRAMMER
    // ^ CHANGED IR[13:12] to mem_size_after FOR PROGRAMMER
    // ^ CHANGED IR[14] to mem_sign_after FOR PROGRAMMER
     
     
     //==== THE FSM WILL HAVE TO BE REPLACED BY DECODERS IN EACH STAGE ====//
     
     //logic prev_INT=0;
     
//     OTTER_CU_FSM CU_FSM (.CU_CLK(CLK), .CU_INT(INTR), .CU_RESET(RESET), .CU_OPCODE(opcode), //.CU_OPCODE(opcode),
//                     .CU_FUNC3(IR[14:12]),.CU_FUNC12(IR[31:20]),
//                     .CU_PCWRITE(pcWrite), .CU_REGWRITE(regWrite), .CU_MEMWRITE(memWrite), 
//                     .CU_MEMREAD1(memRead1),.CU_MEMREAD2(memRead2),.CU_intTaken(intTaken),.CU_intCLR(intCLR),.CU_csrWrite(csrWrite),.CU_prevINT(prev_INT));
    
    
    
//    always_ff @ (posedge CLK)
//    begin
//         if(INTR && mie)
//            prev_INT=1'b1;
//         if(intCLR || RESET)
//            prev_INT=1'b0;
//    end
    //MMIO /////////////////////////////////////////////////////           
    assign IOBUS_ADDR = mem_addr_after;  // CHANGED FROM aluResult TO mem_addr_after FOR PROGRAMMER
    assign IOBUS_OUT = mem_data_after;  // CHANGED FROM B TO mem_data_after FOR PROGRAMMER 
         
            
endmodule
