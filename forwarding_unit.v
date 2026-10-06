`timescale 1ns / 1ps

// ============================================================================
// FORWARDING UNIT
//
// Three forwarding sources are needed in THIS pipeline (not the usual two).
// Reason: the register file is written by the posedge that ENDS the WB cycle,
// and it is read on the negedge in the middle of the ID cycle. Work out which
// producers a consumer can see, for a consumer whose EX cycle is T:
//
//   producer 1 instruction  ahead -> in EX/MEM during T          -> forward 10
//   producer 2 instructions ahead -> in MEM/WB during T          -> forward 01
//   producer 3 instructions ahead -> its RF write lands at T     -> forward 11
//   producer 4+ instructions ahead-> already in the RF at the
//                                    negedge read of cycle T-1   -> no forward
//
// The original two-level unit left the 3-instruction case uncovered, so a
// dependency exactly 3 instructions apart silently read a stale register.
// Source 11 (the register-file write-port shadow) closes that gap.
//
// Priority is newest-first: EX/MEM > MEM/WB > RF-write shadow.
//
// Note on loads: when the producer sitting in EX/MEM is a LW, EX/MEM holds the
// effective ADDRESS, not the loaded data. The hazard unit guarantees that this
// situation never reaches EX by stalling for one cycle, which pushes the load
// into MEM/WB (where wb_write_data is the real loaded value) and leaves a
// bubble in EX/MEM.
// ============================================================================
module forwarding_unit (
    input  [4:0] id_ex_rs,
    input  [4:0] id_ex_rt,

    input  [4:0] ex_mem_rd,
    input        ex_mem_reg_write,

    input  [4:0] mem_wb_rd,
    input        mem_wb_reg_write,

    input  [4:0] rf_wr_rd,            // NEW: destination written at this edge
    input        rf_wr_reg_write,     // NEW

    output reg [1:0] forward_a,       // 00 = RF, 10 = EX/MEM, 01 = MEM/WB, 11 = RF-write
    output reg [1:0] forward_b
);
    always @(*) begin
        // ---------------- Forward A (rs operand) ----------------
        if (ex_mem_reg_write &&
            (ex_mem_rd != 5'd0) &&
            (ex_mem_rd == id_ex_rs))
            forward_a = 2'b10;
        else if (mem_wb_reg_write &&
                 (mem_wb_rd != 5'd0) &&
                 (mem_wb_rd == id_ex_rs))
            forward_a = 2'b01;
        else if (rf_wr_reg_write &&
                 (rf_wr_rd != 5'd0) &&
                 (rf_wr_rd == id_ex_rs))
            forward_a = 2'b11;
        else
            forward_a = 2'b00;

        // ---------------- Forward B (rt operand) ----------------
        if (ex_mem_reg_write &&
            (ex_mem_rd != 5'd0) &&
            (ex_mem_rd == id_ex_rt))
            forward_b = 2'b10;
        else if (mem_wb_reg_write &&
                 (mem_wb_rd != 5'd0) &&
                 (mem_wb_rd == id_ex_rt))
            forward_b = 2'b01;
        else if (rf_wr_reg_write &&
                 (rf_wr_rd != 5'd0) &&
                 (rf_wr_rd == id_ex_rt))
            forward_b = 2'b11;
        else
            forward_b = 2'b00;
    end
endmodule
