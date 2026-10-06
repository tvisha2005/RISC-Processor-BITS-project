`timescale 1ns / 1ps

// ============================================================================
// HAZARD DETECTION UNIT  (load-use stall)
//
// A load produces its data too late for the instruction immediately behind it:
// while the consumer is in EX, the load is in MEM and EX/MEM still holds the
// effective address rather than the loaded word.
//
// One stall cycle fixes it exactly. After the stall the load has advanced to
// MEM/WB (where wb_write_data is the loaded value, selected by mem_to_reg) and
// EX/MEM holds the injected bubble, so the forwarding unit picks source 01.
//
// Effect of the stall signal:
//   - PC and IF/ID hold their current values (instruction is re-decoded)
//   - ID/EX is loaded with a bubble (all control signals cleared)
//
// id_ex_write_reg is the ALREADY-MUXED destination (reg_dst applied), not the
// raw rt field, so the check stays correct regardless of instruction format.
// ============================================================================
module hazard_unit (
    input        id_ex_mem_read,      // instruction in EX is a load
    input  [4:0] id_ex_write_reg,     // its destination register
    input  [4:0] id_rs,               // register fields of the instruction in ID
    input  [4:0] id_rt,
    input        id_uses_rs,          // ...and whether ID actually reads them
    input        id_uses_rt,
    output       stall
);

    assign stall = id_ex_mem_read &&
                   (id_ex_write_reg != 5'd0) &&
                   ((id_uses_rs && (id_ex_write_reg == id_rs)) ||
                    (id_uses_rt && (id_ex_write_reg == id_rt)));

endmodule
