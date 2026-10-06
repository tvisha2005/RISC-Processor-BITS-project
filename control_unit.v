`timescale 1ns / 1ps

// ============================================================================
// CONTROL UNIT
//
// New instructions in this revision: LW, SW, BEQ, BNE
//
// New control outputs:
//   branch      : instruction is a conditional branch (resolved in EX)
//   branch_ne   : 0 = BEQ (branch when equal), 1 = BNE (branch when not equal)
//   sign_ext    : 1 = immediate is sign-extended (LW/SW/BEQ/BNE)
//                 0 = immediate is zero-extended (ORI/LUI)
//   uses_rs     : instruction actually reads the rs register
//   uses_rt     : instruction actually reads the rt register (as a SOURCE)
//
// uses_rs / uses_rt exist purely so the hazard unit does not insert bogus
// stalls for register fields that the instruction never reads. They must be
// exact: claiming a register is unused when it is read would drop a needed
// stall.
// ============================================================================
module control_unit (
    input      [5:0] opcode,
    input      [5:0] funct,

    output reg       reg_write,   // 1 = write result to register file
    output reg       reg_dst,     // 0 = write to rt (I-type), 1 = write to rd (R-type)
    output reg       alu_src,     // 0 = second ALU operand is rt, 1 = immediate
    output reg [2:0] alu_op,

    // Memory control -- now genuinely used by the MEM stage
    output reg       mem_read,
    output reg       mem_write,
    output reg       mem_to_reg,  // 0 = ALU result to WB, 1 = memory data to WB

    output reg       jump,        // unconditional J (resolved in ID)
    output reg       branch,      // NEW: BEQ/BNE (resolved in EX)
    output reg       branch_ne,   // NEW: 0 = BEQ, 1 = BNE
    output reg       sign_ext,    // NEW: immediate extension mode
    output reg       uses_rs,     // NEW: for hazard detection
    output reg       uses_rt      // NEW: for hazard detection
);

    localparam OP_RTYPE = 6'b000000;
    localparam OP_JUMP  = 6'b000010;
    localparam OP_BEQ   = 6'b000100;   // NEW
    localparam OP_BNE   = 6'b000101;   // NEW
    localparam OP_ORI   = 6'b001101;
    localparam OP_LUI   = 6'b001111;
    localparam OP_LW    = 6'b100011;   // NEW
    localparam OP_SW    = 6'b101011;   // NEW
    localparam OP_LSL   = 6'b110010;
    localparam OP_LSR   = 6'b111011;

    localparam FUNCT_SLT = 6'b101010;

    localparam ALU_SLT = 3'b000;
    localparam ALU_LSL = 3'b001;
    localparam ALU_LSR = 3'b010;
    localparam ALU_ORI = 3'b011;
    localparam ALU_LUI = 3'b100;
    localparam ALU_ADD = 3'b101;
    localparam ALU_SUB = 3'b110;

    always @(*) begin
        // Defaults = an inert instruction (this is also what NOP, 32'h0, decodes to)
        reg_write  = 1'b0;
        reg_dst    = 1'b0;
        alu_src    = 1'b0;
        alu_op     = 3'b000;
        mem_read   = 1'b0;
        mem_write  = 1'b0;
        mem_to_reg = 1'b0;
        jump       = 1'b0;
        branch     = 1'b0;
        branch_ne  = 1'b0;
        sign_ext   = 1'b0;
        uses_rs    = 1'b0;
        uses_rt    = 1'b0;

        case (opcode)

            OP_RTYPE: begin
                if (funct == FUNCT_SLT) begin
                    reg_write = 1'b1;
                    reg_dst   = 1'b1;  // destination = rd
                    alu_src   = 1'b0;  // second operand = rt register
                    alu_op    = ALU_SLT;
                    uses_rs   = 1'b1;
                    uses_rt   = 1'b1;
                end
                // funct == 0 (NOP) and any other funct: all controls stay 0
            end

            OP_LSL: begin
                reg_write = 1'b1;
                reg_dst   = 1'b1;  // destination = rd
                alu_src   = 1'b0;  // shift amount from rt register (NOT shamt field)
                alu_op    = ALU_LSL;
                uses_rs   = 1'b1;
                uses_rt   = 1'b1;
            end

            OP_LSR: begin
                reg_write = 1'b1;
                reg_dst   = 1'b1;
                alu_src   = 1'b0;
                alu_op    = ALU_LSR;
                uses_rs   = 1'b1;
                uses_rt   = 1'b1;
            end

            OP_ORI: begin
                reg_write = 1'b1;
                reg_dst   = 1'b0;  // destination = rt
                alu_src   = 1'b1;  // second operand = zero-extended immediate
                alu_op    = ALU_ORI;
                sign_ext  = 1'b0;  // ORI is zero-extended (MIPS behaviour)
                uses_rs   = 1'b1;
                uses_rt   = 1'b0;  // rt is the destination here
            end

            OP_LUI: begin
                reg_write = 1'b1;
                reg_dst   = 1'b0;  // destination = rt
                alu_src   = 1'b1;  // immediate used; rs field is 00000
                alu_op    = ALU_LUI;
                sign_ext  = 1'b0;
                uses_rs   = 1'b0;  // rs field is ignored
                uses_rt   = 1'b0;
            end

            // ---------------- NEW: load word ----------------
            // LW rt, imm(rs)      rt <- MEM[rs + sign_ext(imm)]
            OP_LW: begin
                reg_write  = 1'b1;
                reg_dst    = 1'b0;  // destination = rt
                alu_src    = 1'b1;  // ALU adds the offset
                alu_op     = ALU_ADD;
                mem_read   = 1'b1;
                mem_to_reg = 1'b1;  // WB data comes from memory
                sign_ext   = 1'b1;
                uses_rs    = 1'b1;  // base register
                uses_rt    = 1'b0;  // rt is the destination
            end

            // ---------------- NEW: store word ----------------
            // SW rt, imm(rs)      MEM[rs + sign_ext(imm)] <- rt
            OP_SW: begin
                reg_write  = 1'b0;  // no register destination
                reg_dst    = 1'b0;
                alu_src    = 1'b1;  // ALU adds the offset
                alu_op     = ALU_ADD;
                mem_write  = 1'b1;
                mem_to_reg = 1'b0;
                sign_ext   = 1'b1;
                uses_rs    = 1'b1;  // base register
                uses_rt    = 1'b1;  // rt supplies the STORE DATA
            end

            // ---------------- NEW: branch if equal ----------------
            // BEQ rs, rt, imm     if (rs == rt) PC <- PC+4 + (sign_ext(imm) << 2)
            OP_BEQ: begin
                reg_write = 1'b0;
                alu_src   = 1'b0;  // compare rs against rt, not the immediate
                alu_op    = ALU_SUB;
                branch    = 1'b1;
                branch_ne = 1'b0;
                sign_ext  = 1'b1;  // branch displacement is signed
                uses_rs   = 1'b1;
                uses_rt   = 1'b1;
            end

            // ---------------- NEW: branch if not equal ----------------
            OP_BNE: begin
                reg_write = 1'b0;
                alu_src   = 1'b0;
                alu_op    = ALU_SUB;
                branch    = 1'b1;
                branch_ne = 1'b1;
                sign_ext  = 1'b1;
                uses_rs   = 1'b1;
                uses_rt   = 1'b1;
            end

            OP_JUMP: begin
                jump    = 1'b1;
                uses_rs = 1'b0;
                uses_rt = 1'b0;
            end

            default: ;
        endcase
    end

endmodule
