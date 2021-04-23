module Forwarding_Unit (
  input logic [4:0] RS1, RS2, EX_MEM_RD, MEM_WB_RD, // input register addresses
  input RS1_USED, RS2_USED, EX_MEM_REGWRITE, MEM_WB_REGWRITE, LD_HAZ,
  output logic [1:0] SEL_A, SEL_B // output select bits for ALU MUXes
  );
    
  typedef enum {
    NO_HAZ, // 2'b00
    MEM_HAZ, // 2'b01
    WB_HAZ, // 2'b10
    LOAD_HAZ // 2b'11
  } hazard_t;
  
  // Hazard detection for ALU input A
  always_comb
  begin
    SEL_A = 0;
    if (LD_HAZ && (MEM_WB_RD == RS1)) SEL_A = LOAD_HAZ;
    else if (EX_MEM_REGWRITE && (EX_MEM_RD != 0) && (EX_MEM_RD == RS1)) SEL_A = MEM_HAZ; // take most recent hazard
    else if (MEM_WB_REGWRITE && (MEM_WB_RD != 0) && (MEM_WB_RD == RS1)) SEL_A = WB_HAZ; 
  end
  
  // Hazard detection for ALU input B
  always_comb
  begin
    SEL_B = 0;
    if (LD_HAZ && (MEM_WB_RD == RS2)) SEL_B = LOAD_HAZ;
    if (EX_MEM_REGWRITE && (EX_MEM_RD != 0) && (EX_MEM_RD == RS2)) SEL_B = MEM_HAZ; // take most recent hazard
    else if (MEM_WB_REGWRITE && (MEM_WB_RD != 0) && (MEM_WB_RD == RS2)) SEL_B = WB_HAZ;
    
  end
endmodule //
