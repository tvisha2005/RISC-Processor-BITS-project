`timescale 1ns / 1ps

// ============================================================================
// REGISTER FILE  --  behaviour unchanged from the previous version
//
// Timing, spelled out because the forwarding network depends on it:
//   write : posedge, from the MEM/WB registers. The write therefore lands at
//           the edge that ENDS the WB cycle, one edge later than a classic
//           mid-cycle writeback.
//   read  : negedge, i.e. halfway through the ID cycle, into rs_data/rt_data.
//
// Consequence: an instruction in ID sees only the writes of producers that are
// 4 or more instructions ahead of it. Producers 1, 2 and 3 instructions ahead
// must all be forwarded -- see forwarding_unit.v (three sources, not two).
// ============================================================================
module register_file (
    input             clk,
    input             reset,
    input      [4:0]  rs,
    input      [4:0]  rt,
    output reg [31:0] rs_data,
    output reg [31:0] rt_data,
    input      [4:0]  rd,
    input      [31:0] write_data,
    input             reg_write
);
    reg [31:0] regs [0:31];
    integer    k;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (k = 0; k < 32; k = k + 1)
                regs[k] <= 32'd0;
            regs[2]  <= 32'd8;
            regs[4]  <= 32'd16;
            regs[7]  <= 32'd64;
            regs[8]  <= 32'd2;
            regs[9]  <= 32'd3;
            regs[10] <= 32'd1;
        end else begin
            if (reg_write && rd != 5'd0)
                regs[rd] <= write_data;
            regs[0] <= 32'd0; // R0 always zero
        end
    end

    always @(negedge clk or posedge reset) begin
        if (reset) begin
            rs_data <= 32'd0;
            rt_data <= 32'd0;
        end else begin
            rs_data <= (rs == 5'd0) ? 32'd0 : regs[rs];
            rt_data <= (rt == 5'd0) ? 32'd0 : regs[rt];
        end
    end
endmodule
