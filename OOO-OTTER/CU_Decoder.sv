`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: J. Callenes
// 
// Create Date: 01/27/2019 09:22:55 AM
// Design Name: 
// Module Name: CU_Decoder
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
//`include "opcodes.svh"

module OTTER_CU_Decoder(
    input [6:0] CU_OPCODE_0, 
    input [6:0] CU_OPCODE_1,
    input [2:0] CU_FUNC3_0, 
    input [2:0] CU_FUNC3_1,
    input [6:0] CU_FUNC7_0,
    input [6:0] CU_FUNC7_1,
    input intTaken,
    output logic CU_ALU_SRCA_0,
    output logic CU_ALU_SRCA_1,
    output logic [1:0] CU_ALU_SRCB_0, 
    output logic [1:0] CU_ALU_SRCB_1,
    output logic [3:0] CU_ALU_FUN_0,
    output logic [3:0]  CU_ALU_FUN_1,
    output logic [1:0] CU_RF_WR_SEL_0, 
    output logic [1:0] CU_RF_WR_SEL_1
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
        
        
        typedef enum logic [2:0] {
                Func3_CSRRW  = 3'b001,
                Func3_CSRRS  = 3'b010,
                Func3_CSRRC  = 3'b011,
                Func3_CSRRWI = 3'b101,
                Func3_CSRRSI = 3'b110,
                Func3_CSRRCI = 3'b111,
                Func3_PRIV   = 3'b000       //mret
        } funct3_system_t;

       
        opcode_t OPCODE_0, OPCODE_1;
        assign OPCODE_0 = opcode_t'(CU_OPCODE_0);
        assign OPCODE_1 = opcode_t'(CU_OPCODE_1);
        
       //DECODING  (does not depend on state)  ////////////////////////////////////////////
       //SEPERATE DECODER
       
        always_comb // instruction 0 decoding
        begin
            case(CU_OPCODE_0)
                OP_IMM: CU_ALU_FUN_0= (CU_FUNC3_0==3'b101)?{CU_FUNC7_0[5],CU_FUNC3_0}:{1'b0,CU_FUNC3_0};
                LUI,SYSTEM: CU_ALU_FUN_0 = 4'b1001;
                OP: CU_ALU_FUN_0 = {CU_FUNC7_0[5],CU_FUNC3_0};
                default: CU_ALU_FUN_0 = 4'b0;
            endcase
        end
        
        always_comb
         begin
            //if(state==1 || state==2)
                case(CU_OPCODE_0)
                    JAL:    CU_RF_WR_SEL_0=0;
                    JALR:    CU_RF_WR_SEL_0=0;
                    LOAD:    CU_RF_WR_SEL_0=2;
                    SYSTEM:  CU_RF_WR_SEL_0=1;
                    default: CU_RF_WR_SEL_0=3; 
                endcase
            //else CU_RF_WR_SEL=3;   
          end   
          
         always_comb
         begin
         // if(state!=0)
            case(CU_OPCODE_0)
                STORE:  CU_ALU_SRCB_0=2;  //S-type
                LOAD:   CU_ALU_SRCB_0=1;  //I-type
                JAL:    CU_ALU_SRCB_0=1;  //I-type
                OP_IMM: CU_ALU_SRCB_0=1;  //I-type
                AUIPC:  CU_ALU_SRCB_0=3;  // U-type (special) LUI does not use B
                default:CU_ALU_SRCB_0=0;  //R-type    //OP  BRANCH-does not use
            endcase
          //else CU_ALU_SRCB=3;
         end
           
       assign CU_ALU_SRCA_0 = (CU_OPCODE_0==LUI || CU_OPCODE_0==AUIPC) ? 1 : 0;
       
        
        always_comb // instruction 1 decoding
        begin
            case(CU_OPCODE_1)
                OP_IMM: CU_ALU_FUN_1= (CU_FUNC3_1==3'b101)?{CU_FUNC7_1[5],CU_FUNC3_1}:{1'b0,CU_FUNC3_1};
                LUI,SYSTEM: CU_ALU_FUN_1 = 4'b1001;
                OP: CU_ALU_FUN_1 = {CU_FUNC7_1[5],CU_FUNC3_1};
                default: CU_ALU_FUN_1 = 4'b0;
            endcase
        end
           
        always_comb
         begin
            //if(state==1 || state==2)
                case(CU_OPCODE_1)
                    JAL:    CU_RF_WR_SEL_1=0;
                    JALR:    CU_RF_WR_SEL_1=0;
                    LOAD:    CU_RF_WR_SEL_1=2;
                    SYSTEM:  CU_RF_WR_SEL_1=1;
                    default: CU_RF_WR_SEL_1=3; 
                endcase
            //else CU_RF_WR_SEL=3;   
          end   
          
          
         always_comb
         begin
         // if(state!=0)
            case(CU_OPCODE_1)
                STORE:  CU_ALU_SRCB_1=2;  //S-type
                LOAD:   CU_ALU_SRCB_1=1;  //I-type
                JAL:    CU_ALU_SRCB_1=1;  //I-type
                OP_IMM: CU_ALU_SRCB_1=1;  //I-type
                AUIPC:  CU_ALU_SRCB_1=3;  // U-type (special) LUI does not use B
                default:CU_ALU_SRCB_1=0;  //R-type    //OP  BRANCH-does not use
            endcase
          //else CU_ALU_SRCB=3;
         end
           
       assign CU_ALU_SRCA_1 = (CU_OPCODE_1==LUI || CU_OPCODE_1==AUIPC) ? 1 : 0;
                
        //assign CU_MSIZE = CU_FUNC3[1:0];        

endmodule
