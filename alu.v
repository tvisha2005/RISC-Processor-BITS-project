`timescale 1ns / 1ps

// ============================================================================
// ALU
//   Added for this revision:
//     ALU_ADD  -> effective-address computation for LW / SW
//     ALU_SUB  -> operand comparison for BEQ / BNE (via the zero flag)
//     zero     -> asserted when result == 0
// ============================================================================
module alu (
    input      [31:0] a,
    input      [31:0] b,
    input      [2:0]  alu_op,
    output reg [31:0] result,
    output            zero
);
    localparam ALU_SLT = 3'b000;
    localparam ALU_LSL = 3'b001;
    localparam ALU_LSR = 3'b010;
    localparam ALU_ORI = 3'b011;
    localparam ALU_LUI = 3'b100;
    localparam ALU_ADD = 3'b101;   // NEW
    localparam ALU_SUB = 3'b110;   // NEW

    always @(*) begin
        case (alu_op)
            ALU_SLT: result = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
            ALU_LSL: result = a << b[4:0];
            ALU_LSR: result = a >> b[4:0];
            ALU_ORI: result = a | b;
            ALU_LUI: result = b << 16;
            ALU_ADD: result = a + b;
            ALU_SUB: result = a - b;
            default: result = 32'd0;
        endcase
    end

    // For BEQ/BNE the ALU is told to do SUB, so zero == (rs == rt).
    assign zero = (result == 32'd0);

endmodule
