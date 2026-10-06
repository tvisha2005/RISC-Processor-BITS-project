`timescale 1ns / 1ps

// ============================================================================
// INSTRUCTION FETCH STAGE  (PC + instruction memory + IF/ID register)
//
// New control inputs:
//   stall       : hold PC and hold IF/ID. The instruction currently in ID is
//                 decoded again next cycle; ID/EX gets a bubble instead.
//   redirect    : take redirect_pc as the next PC and drop the instruction
//                 being fetched this cycle (IF/ID <= NOP). Driven by a taken
//                 branch (mispredict recovery) or by J.
//
// Priority: reset > redirect > stall > sequential fetch.
// Redirect must outrank stall: if a branch resolves taken, whatever sits in
// ID belongs to the wrong path and must be discarded, not stalled.
//
// The instruction latched into IF/ID is imem[PC], and if_id_pc4 is PC+4, so
// if_id_pc4 is the fall-through address of the instruction in ID. That value
// is carried down to EX and used as the branch base address.
// ============================================================================
module instruction_fetch (
    input             clk,
    input             reset,
    input             stall,
    input             redirect,
    input      [31:0] redirect_pc,
    output reg [31:0] if_id_instr,
    output reg [31:0] if_id_pc4
);
    localparam NOP = 32'h0000_0000;   // decodes to an inert R-type

    reg [31:0] PC;
    reg [31:0] imem [0:63];
    integer i;

    // ---- opcode / funct encodings (must match control_unit.v) ----
    localparam OP_RTYPE = 6'b000000;
    localparam OP_JUMP  = 6'b000010;
    localparam OP_BEQ   = 6'b000100;
    localparam OP_BNE   = 6'b000101;
    localparam OP_ORI   = 6'b001101;
    localparam OP_LUI   = 6'b001111;
    localparam OP_LW    = 6'b100011;
    localparam OP_SW    = 6'b101011;
    localparam OP_LSL   = 6'b110010;
    localparam OP_LSR   = 6'b111011;
    localparam F_SLT    = 6'b101010;
    localparam F_NONE   = 6'b000000;

    // ========================================================================
    // TEST PROGRAM
    //
    // Reset state of the register file:
    //   r2=8  r4=16  r7=64  r8=2  r9=3  r10=1   (all others 0)
    // Data memory: all zero except word 63 (byte address 252) = 0xCAFEBABE
    //
    // idx  addr  instruction                       expected effect
    // ---  ----  --------------------------------  ---------------------------
    // ORIGINAL FEATURE REGRESSION (unchanged from the previous version)
    //   0     0  lsl  r1, r2, r8                   r1 = 8 << 2   = 32
    //   1     4  lsl  r3, r4, r9                   r3 = 16 << 3  = 128
    //   2     8  slt  r5, r1, r3                   r5 = (32<128) = 1   [fwd 10 + 01]
    //   3    12  ori  r6, r5, 200                  r6 = 1|200    = 201 [fwd 10]
    //   4    16  lsr  r6, r7, r10                  r6 = 64 >> 1  = 32
    //   5    20  j    7                            jump over idx 6
    //   6    24  lsl  r7, r7, r8                   MUST NOT EXECUTE
    //   7    28  lui  r6, 128                      r6 = 0x00800000
    // LOAD / STORE
    //   8    32  ori  r11, r0, 0x1234              r11 = 0x1234
    //   9    36  sw   r11, 0(r0)                   mem[0]  = 0x1234  [store data fwd]
    //  10    40  lw   r12, 0(r0)                   r12 = 0x1234  (load right after store)
    //  11    44  ori  r13, r12, 0                  LOAD-USE STALL -> r13 = 0x1234
    //  12    48  sw   r13, 4(r0)                   mem[4]  = 0x1234
    //  13    52  lw   r14, 4(r0)                   r14 = 0x1234
    //  14    56  sw   r14, 8(r0)                   LOAD-USE STALL (rt = store data)
    //  15    60  lw   r15, 8(r0)                   r15 = 0x1234
    // BRANCH: correctly predicted (not taken)
    //  16    64  ori  r16, r0, 5                   r16 = 5
    //  17    68  ori  r17, r0, 5                   r17 = 5
    //  18    72  ori  r18, r0, 9                   r18 = 9
    //  19    76  beq  r16, r18, +2                 5 != 9 -> NOT taken, no flush
    //  20    80  ori  r19, r0, 1                   executes -> r19 = 1
    //  21    84  bne  r16, r17, +2                 5 == 5 -> NOT taken, no flush
    //  22    88  ori  r20, r0, 2                   executes -> r20 = 2
    // BRANCH: mispredicted (taken) -> flush
    //  23    92  beq  r16, r17, +3                 TAKEN -> target = 96+12 = 108
    //  24    96  ori  r21, r0, 0x1111              KILLED (in ID at resolve)
    //  25   100  sw   r16, 12(r0)                  KILLED (mem[12] must stay 0)
    //  26   104  ori  r22, r0, 0x2222              SKIPPED (branched over)
    //  27   108  ori  r23, r0, 7                   r23 = 7
    //  28   112  bne  r16, r18, +2                 TAKEN -> target = 116+8 = 124
    //  29   116  ori  r24, r0, 0x3333              KILLED
    //  30   120  sw   r18, 16(r0)                  KILLED (mem[16] must stay 0)
    //  31   124  ori  r25, r0, 8                   r25 = 8
    // BRANCH whose operand comes from a load -> stall AND flush
    //  32   128  lw   r26, 0(r0)                   r26 = 0x1234
    //  33   132  beq  r26, r12, +2                 STALL, then TAKEN -> 136+8 = 144
    //  34   136  ori  r27, r0, 0x4444              KILLED
    //  35   140  ori  r28, r0, 0x5555              KILLED
    //  36   144  ori  r29, r0, 9                   r29 = 9
    // BRANCH whose operands both come from forwarding
    //  37   148  ori  r30, r0, 9                   r30 = 9
    //  38   152  beq  r30, r29, +2                 fwd 10 (r30) + 01 (r29) -> TAKEN
    //  39   156  sw   r30, 20(r0)                  KILLED (mem[20] must stay 0)
    //  40   160  ori  r31, r0, 0x6666              KILLED (r31 must stay 0)
    //  41   164  ori  r2,  r0, 0x0ABC              r2 = 0x0ABC
    // BACKWARD BRANCH (negative, sign-extended offset) = a real loop
    //  42   168  ori  r5, r0, 8                    loop counter
    //  43   172  ori  r6, r0, 1                    shift amount
    //  44   176  ori  r7, r0, 1                    iteration marker
    //  45   180  lsr  r5, r5, r6                   r5 >>= 1   [needs fwd 11: r5 is 3 back]
    //  46   184  lsl  r7, r7, r6                   r7 <<= 1
    //  47   188  bne  r5, r0, -3                   loop back to 180 while r5 != 0
    //                                              4 passes: r5 8->0, r7 1->16
    //  48   192  sw   r7, 24(r0)                   mem[24] = 16
    //  49   196  lw   r8, 24(r0)                   r8 = 16
    //  50   200  slt  r9, r8, r3                   LOAD-USE STALL -> r9 = (16<128) = 1
    //  51   204  sw   r9, 28(r0)                   mem[28] = 1
    //  52   208  lw   r4, 252(r0)                  r4 = 0xCAFEBABE (pre-loaded word)
    //  53   212  ori  r1, r4, 0                    LOAD-USE STALL -> r1 = 0xCAFEBABE
    //  54   216  j    54                           halt (jump to self)
    // ========================================================================
    initial begin
        for (i = 0; i < 64; i = i + 1)
            imem[i] = NOP;

        // ---- original program ----
        imem[0]  = {OP_LSL,   5'd2,  5'd8,  5'd1,  5'd0, F_NONE};
        imem[1]  = {OP_LSL,   5'd4,  5'd9,  5'd3,  5'd0, F_NONE};
        imem[2]  = {OP_RTYPE, 5'd1,  5'd3,  5'd5,  5'd0, F_SLT };
        imem[3]  = {OP_ORI,   5'd5,  5'd6,  16'd200};
        imem[4]  = {OP_LSR,   5'd7,  5'd10, 5'd6,  5'd0, F_NONE};
        imem[5]  = {OP_JUMP,  26'd7};
        imem[6]  = {OP_LSL,   5'd7,  5'd8,  5'd7,  5'd0, F_NONE};
        imem[7]  = {OP_LUI,   5'd0,  5'd6,  16'd128};

        // ---- load / store ----
        imem[8]  = {OP_ORI,   5'd0,  5'd11, 16'h1234};
        imem[9]  = {OP_SW,    5'd0,  5'd11, 16'd0};
        imem[10] = {OP_LW,    5'd0,  5'd12, 16'd0};
        imem[11] = {OP_ORI,   5'd12, 5'd13, 16'd0};
        imem[12] = {OP_SW,    5'd0,  5'd13, 16'd4};
        imem[13] = {OP_LW,    5'd0,  5'd14, 16'd4};
        imem[14] = {OP_SW,    5'd0,  5'd14, 16'd8};
        imem[15] = {OP_LW,    5'd0,  5'd15, 16'd8};

        // ---- branches predicted correctly (not taken) ----
        imem[16] = {OP_ORI,   5'd0,  5'd16, 16'd5};
        imem[17] = {OP_ORI,   5'd0,  5'd17, 16'd5};
        imem[18] = {OP_ORI,   5'd0,  5'd18, 16'd9};
        imem[19] = {OP_BEQ,   5'd16, 5'd18, 16'd2};
        imem[20] = {OP_ORI,   5'd0,  5'd19, 16'd1};
        imem[21] = {OP_BNE,   5'd16, 5'd17, 16'd2};
        imem[22] = {OP_ORI,   5'd0,  5'd20, 16'd2};

        // ---- branches mispredicted (taken) -> flush ----
        imem[23] = {OP_BEQ,   5'd16, 5'd17, 16'd3};
        imem[24] = {OP_ORI,   5'd0,  5'd21, 16'h1111};
        imem[25] = {OP_SW,    5'd0,  5'd16, 16'd12};
        imem[26] = {OP_ORI,   5'd0,  5'd22, 16'h2222};
        imem[27] = {OP_ORI,   5'd0,  5'd23, 16'd7};
        imem[28] = {OP_BNE,   5'd16, 5'd18, 16'd2};
        imem[29] = {OP_ORI,   5'd0,  5'd24, 16'h3333};
        imem[30] = {OP_SW,    5'd0,  5'd18, 16'd16};
        imem[31] = {OP_ORI,   5'd0,  5'd25, 16'd8};

        // ---- branch operand from a load: stall then flush ----
        imem[32] = {OP_LW,    5'd0,  5'd26, 16'd0};
        imem[33] = {OP_BEQ,   5'd26, 5'd12, 16'd2};
        imem[34] = {OP_ORI,   5'd0,  5'd27, 16'h4444};
        imem[35] = {OP_ORI,   5'd0,  5'd28, 16'h5555};
        imem[36] = {OP_ORI,   5'd0,  5'd29, 16'd9};

        // ---- branch operands from forwarding ----
        imem[37] = {OP_ORI,   5'd0,  5'd30, 16'd9};
        imem[38] = {OP_BEQ,   5'd30, 5'd29, 16'd2};
        imem[39] = {OP_SW,    5'd0,  5'd30, 16'd20};
        imem[40] = {OP_ORI,   5'd0,  5'd31, 16'h6666};
        imem[41] = {OP_ORI,   5'd0,  5'd2,  16'h0ABC};

        // ---- backward branch loop ----
        imem[42] = {OP_ORI,   5'd0,  5'd5,  16'd8};
        imem[43] = {OP_ORI,   5'd0,  5'd6,  16'd1};
        imem[44] = {OP_ORI,   5'd0,  5'd7,  16'd1};
        imem[45] = {OP_LSR,   5'd5,  5'd6,  5'd5,  5'd0, F_NONE};
        imem[46] = {OP_LSL,   5'd7,  5'd6,  5'd7,  5'd0, F_NONE};
        imem[47] = {OP_BNE,   5'd5,  5'd0,  16'hFFFD};   // -3 words

        // ---- final load/store + load-use checks ----
        imem[48] = {OP_SW,    5'd0,  5'd7,  16'd24};
        imem[49] = {OP_LW,    5'd0,  5'd8,  16'd24};
        imem[50] = {OP_RTYPE, 5'd8,  5'd3,  5'd9,  5'd0, F_SLT};
        imem[51] = {OP_SW,    5'd0,  5'd9,  16'd28};
        imem[52] = {OP_LW,    5'd0,  5'd4,  16'd252};
        imem[53] = {OP_ORI,   5'd4,  5'd1,  16'd0};

        // ---- halt: jump to self ----
        imem[54] = {OP_JUMP,  26'd54};
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC          <= 32'h0;
            if_id_instr <= NOP;
            if_id_pc4   <= 32'h0;
        end else if (redirect) begin
            // Taken branch or jump: retarget the PC and kill the instruction
            // that was being fetched this cycle.
            PC          <= redirect_pc;
            if_id_instr <= NOP;
            if_id_pc4   <= 32'h0;
        end else if (stall) begin
            // Freeze the front end for one cycle (load-use hazard).
            PC          <= PC;
            if_id_instr <= if_id_instr;
            if_id_pc4   <= if_id_pc4;
        end else begin
            if_id_instr <= imem[PC[7:2]];   // PC[7:2] keeps the index in range
            if_id_pc4   <= PC + 32'd4;
            PC          <= PC + 32'd4;
        end
    end
endmodule
