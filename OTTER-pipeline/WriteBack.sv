import cpu_types::*;

module WriteBack (
    input CLK,
    input [31:0] mem_data, WB_aluResult,
    input instr_t MEM_WB_instr,

    output [31:0] WB_rfIn 
);

    logic [31:0] MEM_PC;
    assign MEM_PC = MEM_WB_instr.pc + 4;
    
    Mult4to1 regWriteback (MEM_PC,0, mem_data, WB_aluResult, MEM_WB_instr.rf_wr_sel, WB_rfIn);
    
endmodule