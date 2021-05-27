import cpu_types::*;

module Memory (
    input CLK, ld_haz, memRead1, mem_sign_after,
    input [31:0] fetch_pc_0, fetch_pc_1, MEM_aluResult, MEM_DIN2, IOBUS_IN,
    input instr_t EX_MEM_instr,

    output [31:0] IR_0, IR_1, mem_data, 
    output IOBUS_WR
);
     // Sets up memory for the fetch stage and for memory accessing in Memory stage
    // In the future need to check on IO and Programmer stuff
    OTTER_mem_byte #(14) memory  (.MEM_CLK(CLK),.MEM_ADDR1_0(fetch_pc_0), .MEM_ADDR1_1(fetch_pc_1),.MEM_ADDR2(MEM_aluResult),.MEM_DIN2(MEM_DIN2),
                               .MEM_WRITE2(EX_MEM_instr.memWrite),.MEM_READ1(memRead1),.MEM_READ2(EX_MEM_instr.memRead2), .LD_HAZ(ld_haz),
                               .ERR(),.MEM_DOUT1_0(IR_0), .MEM_DOUT1_1(IR_1),.MEM_DOUT2(mem_data),.IO_IN(IOBUS_IN),.IO_WR(IOBUS_WR),.MEM_SIZE(EX_MEM_instr.mem_type[1:0]),.MEM_SIGN(mem_sign_after));

endmodule