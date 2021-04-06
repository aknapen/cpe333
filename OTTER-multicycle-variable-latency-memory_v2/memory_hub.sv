`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Keefe Johnson
//              Joseph Callenes
// Create Date: 02/06/2020 06:40:37 PM
// Updated Date: 06/10/2020 08:00:00 AM
// Design Name: 
// Module Name: memory_hub
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - Adapted for AXI-lite bus
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


import memory_bus_sizes::*;

module memory_hub(
    input clk,
    output err,
    axi_bus_ro.device cpu1,
    axi_bus_rw.device cpu2,
    axi_bus_rw.device prog,
    axi_bus_rw.controller mmio,
    axi_bus_ro.controller memory1,
    axi_bus_rw.controller memory2
    );

    axi_bus_rw  cpu_translate();
    axi_bus_rw  mhub2();
    logic s_trans_err;

    size_translator trans(
        .clk(clk), .cpu2(cpu2), .cpu_sizeTrans(cpu_translate), .err(s_trans_err)
    );
    merge_controllers merge(
        .clk(clk), .cpu_sizeTrans(cpu_translate), .prog(prog), .mhub(mhub2)
    );
    split_devices split(
        .clk(clk), .mhub(mhub2), .mmio(mmio), .memory2(memory2)
    );

    //Read Port (1) 
    always_comb begin
        //CPU to Memory
        memory1.read_addr = cpu1.read_addr;
        memory1.read_addr_valid = cpu1.read_addr_valid;
        //Memory to CPU
        cpu1.read_addr_ready = memory1.read_addr_ready;
        cpu1.read_data = memory1.read_data;
        cpu1.read_data_valid = memory1.read_data_valid;
    end
    
        
endmodule

module size_translator(
    input clk,
    axi_bus_rw.device cpu2,
    axi_bus_rw.controller cpu_sizeTrans,
    output err
    );

    typedef enum logic [1:0] {B=2'b00, H=2'b01, W=2'b10, D=2'b11} t_size;

    logic [$clog2(WORD_SIZE)-1:0] r_read_data_byte_sel = '0;
    logic [1:0] r_read_data_size = '0;
    logic r_read_data_lu = '0;

    logic [$clog2(WORD_SIZE)-1:0] s_byte_sel;
    logic s_write_data_err, s_read_data_err;
    logic s_sign;
    
    assign s_byte_sel = cpu2.read_addr[$clog2(WORD_SIZE)-1:0];
 
    assign err = s_write_data_err || s_read_data_err;

    // this module should work for both 32-bit and 64-bit RISC-V
    // the synthesizer should optimize out the 64-bit parts when WORD_SIZE < 8

    always_comb begin
        //CPU to Memory
        cpu_sizeTrans.read_addr = cpu2.read_addr;
        cpu_sizeTrans.read_addr_valid = cpu2.read_addr_valid;
        cpu_sizeTrans.write_addr = cpu2.write_addr;
        cpu_sizeTrans.write_addr_valid = cpu2.write_addr_valid;
        cpu_sizeTrans.write_data = cpu2.write_data;
        //Memory to CPU
        cpu2.read_addr_ready = cpu_sizeTrans.read_addr_ready;
        cpu2.read_data = cpu_sizeTrans.read_data;
        cpu2.read_data_valid = cpu_sizeTrans.read_data_valid;
        cpu2.write_addr_ready = cpu_sizeTrans.write_addr_ready;

        // translate din for write ops (stores)
        cpu_sizeTrans.strobe = '0;
        cpu_sizeTrans.write_data = '0;
        s_write_data_err = cpu2.write_addr_valid;  // if a write op, assume error unless acceptable size and byte_sel
        case (t_size'(cpu2.size))
            B: begin
                cpu_sizeTrans.strobe[s_byte_sel] = 'b1;
                cpu_sizeTrans.write_data[s_byte_sel*8+:8] = cpu2.write_data[7:0];
                s_write_data_err = 0;
            end
            H: if (WORD_SIZE >= 2 && (s_byte_sel & 'b1) == 0) begin
                cpu_sizeTrans.strobe[s_byte_sel+:2] = 'b11;
                cpu_sizeTrans.write_data[s_byte_sel*8+:16] = cpu2.write_data[15:0];
                s_write_data_err = 0;
            end
            W: if (WORD_SIZE >= 4 && (s_byte_sel & 'b11) == 0) begin
                cpu_sizeTrans.strobe[s_byte_sel+:4] = 'b1111;
                cpu_sizeTrans.write_data[s_byte_sel*8+:32] = cpu2.write_data[31:0];
                s_write_data_err = 0;
            end
            D: if (WORD_SIZE >= 8 && (s_byte_sel & 'b111) == 0) begin
                cpu_sizeTrans.strobe[s_byte_sel+:8] = 'b11111111;
                cpu_sizeTrans.write_data[s_byte_sel*8+:64] = cpu2.write_data[63:0];
                s_write_data_err = 0;
            end
        endcase

        // translate dout for read ops (loads)
        cpu2.read_data = '0;
        s_read_data_err = 1;  // assume error unless acceptable size and byte_sel
        case (t_size'(r_read_data_size))
            B: begin
                if (r_read_data_lu) cpu2.read_data = unsigned'(cpu_sizeTrans.read_data[r_read_data_byte_sel*8+:8]);
                else cpu2.read_data = signed'(cpu_sizeTrans.read_data[r_read_data_byte_sel*8+:8]);
                s_read_data_err = 0;
            end
            H: if (WORD_SIZE >= 2 && (r_read_data_byte_sel & 'b1) == 0) begin
                if (r_read_data_lu) cpu2.read_data = unsigned'(cpu_sizeTrans.read_data[r_read_data_byte_sel*8+:16]);
                else cpu2.read_data = signed'(cpu_sizeTrans.read_data[r_read_data_byte_sel*8+:16]);
                s_read_data_err = 0;
            end
            W: if (WORD_SIZE >= 4 && (r_read_data_byte_sel & 'b11) == 0) begin
                if (r_read_data_lu) cpu2.read_data = unsigned'(cpu_sizeTrans.read_data[r_read_data_byte_sel*8+:32]);
                else cpu2.read_data = signed'(cpu_sizeTrans.read_data[r_read_data_byte_sel*8+:32]);
                s_read_data_err = 0;
            end
            D: if (WORD_SIZE >= 8 && (r_read_data_byte_sel & 'b111) == 0) begin
                if (r_read_data_lu) cpu2.read_data = unsigned'(cpu_sizeTrans.read_data[r_read_data_byte_sel*8+:64]);
                else cpu2.read_data = signed'(cpu_sizeTrans.read_data[r_read_data_byte_sel*8+:64]);
                s_read_data_err = 0;
            end
        endcase
    end
    
    // latch info for translating read_data at the same time read_data is latched by its source
    always_ff @(posedge clk) begin
        if (cpu2.read_addr_valid && cpu2.read_addr_ready) begin
            r_read_data_byte_sel <= s_byte_sel; 
            r_read_data_size <= cpu2.size;
            r_read_data_lu <= cpu2.lu;
        end
    end 
    
endmodule

module merge_controllers(
    input clk,
    axi_bus_rw.device cpu_sizeTrans,
    axi_bus_rw.device prog,
    axi_bus_rw.controller mhub
    );
    // prog has priority over cpu_sizeTrans when both start issuing a command in the same
    //   cycle, but operations won't be interrupted once in progress

    // NOTE: implementing the no-change rule for read_data would be non-trivial if both
    //   controllers had the ability to issue read commands, but for now, prog can
    //   only write and has no read_data

    // TODO: determine how prog can trigger icache flush also

    typedef enum {ACCEPTING, PROG_ACTIVE, CPUDT_ACTIVE} t_state;
    t_state r_state = ACCEPTING;
    t_state s_next_state;

    always_comb begin
        s_next_state = r_state;
        prog.write_addr_ready = prog.write_addr_valid;  // by default, hold all commands unless below logic says otherwise
        cpu_sizeTrans.write_addr_ready = mhub.write_addr_ready; //cpu_sizeTrans.write_addr_valid;  // by default, hold all commands unless below logic says otherwise
        cpu_sizeTrans.read_addr_ready = mhub.read_addr_ready;//cpu_sizeTrans.read_addr_valid;  // by default, hold all commands unless below logic says otherwise
        cpu_sizeTrans.read_data = mhub.read_data;  // prog can't read, so mhub.dout should only change on cpudt reads 
        cpu_sizeTrans.read_data_valid = mhub.read_data_valid;
        //mhub = prog;
        mhub.read_addr = prog.read_addr;
        mhub.read_addr_valid = prog.read_addr_valid;
        mhub.strobe = prog.strobe;
        mhub.write_addr = prog.write_addr;
        mhub.write_data = prog.write_data;
        mhub.write_addr_valid = prog.write_addr_valid;
        cpu_sizeTrans.read_data = mhub.read_data;
              
        if (r_state == PROG_ACTIVE || (r_state == ACCEPTING && prog.write_addr_valid)) begin
            prog.write_addr_ready = mhub.write_addr_ready;
            if (!mhub.write_addr_ready) begin
                s_next_state = PROG_ACTIVE;
            end else begin
                s_next_state = ACCEPTING;
            end
        end else if (r_state == CPUDT_ACTIVE || (r_state == ACCEPTING && (cpu_sizeTrans.read_addr_valid || cpu_sizeTrans.write_addr_valid))) begin

            cpu_sizeTrans.read_addr_ready = mhub.read_addr_ready;
            cpu_sizeTrans.write_addr_ready = mhub.write_addr_ready;
            
            mhub.read_addr = cpu_sizeTrans.read_addr;
            mhub.write_addr = cpu_sizeTrans.write_addr;
            mhub.strobe = cpu_sizeTrans.strobe;
            mhub.read_addr_valid = cpu_sizeTrans.read_addr_valid;
            mhub.write_addr_valid = cpu_sizeTrans.write_addr_valid;
            mhub.write_data = cpu_sizeTrans.write_data;
            
            if (!mhub.read_addr_ready || !mhub.write_addr_ready) begin
                s_next_state = CPUDT_ACTIVE;
            end else begin
                s_next_state = ACCEPTING;
            end
        end
    end
    
    always_ff @(posedge clk) begin
        r_state <= s_next_state;
    end

endmodule

module split_devices(
    input clk,
    
    axi_bus_rw.device mhub,
    axi_bus_rw.controller mmio,
    axi_bus_rw.controller memory2
    );

    typedef enum {MEM, MMIO} t_sel_device;

    t_sel_device r_dout_sel_device = MEM;
    t_sel_device s_sel_device;
    always_comb begin
        s_sel_device =  MEM;
        if(mhub.read_addr_valid) begin
            s_sel_device = mhub.read_addr < MMIO_START_ADDR[ADDR_WIDTH-1:0] ? MEM : MMIO;
        end
        if(mhub.write_addr_valid) begin
            s_sel_device = mhub.write_addr < MMIO_START_ADDR[ADDR_WIDTH-1:0] ? MEM: MMIO;
        end   
    end
    assign memory2.read_addr = mhub.read_addr;
    assign memory2.write_addr = mhub.write_addr;
    assign memory2.read_addr_valid = (s_sel_device == MEM) && mhub.read_addr_valid;
    assign memory2.write_addr_valid = (s_sel_device == MEM) && mhub.write_addr_valid;
    assign memory2.write_data = mhub.write_data;
    
    assign mmio.write_addr = mhub.write_addr;
    assign mmio.read_addr = mhub.read_addr;
    assign mmio.read_addr_valid = s_sel_device == MMIO && mhub.read_addr_valid;
    assign mmio.write_addr_valid = s_sel_device == MMIO && mhub.write_addr_valid;
    assign mmio.write_data = mhub.write_data;
    assign mmio.strobe = mhub.strobe;
    
    assign mhub.read_data = (r_dout_sel_device == MEM) ? memory2.read_data : mmio.read_data;
    assign mhub.read_addr_ready = (s_sel_device == MEM) ? memory2.read_addr_ready : mmio.read_addr_ready;
    assign mhub.write_addr_ready = (s_sel_device == MEM) ? memory2.write_addr_ready : mmio.write_addr_ready;
    assign mhub.read_data_valid = (r_dout_sel_device == MEM) ? memory2.read_data_valid : mmio.read_data_valid;
  
    always_ff @(posedge clk) begin
        if (mhub.read_addr_valid && mhub.read_addr_ready) begin //!mhub.we && !mhub.hold) begin
            r_dout_sel_device <= s_sel_device;
        end
    end    
    
endmodule
