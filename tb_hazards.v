`timescale 1ns / 1ps

// ============================================================================
// CORNER-CASE TESTBENCH
//
// Loads a second program directly into the instruction memory (during reset,
// before the first fetch) to hit cases the main program does not reach:
//
//   * RAW dependency exactly 3 instructions apart  -- the gap that needs the
//     third forwarding source; without it this reads a stale register
//   * a load value consumed 2 instructions later, right after a stall
//   * back-to-back independent loads (must NOT stall)
//   * a store whose base AND data register are the same forwarded register
//   * a taken branch landing on another taken branch (two flushes in a row)
//   * a wrong-path LOAD (not just a store) that must not write its register
//   * a taken branch whose target instruction is itself a load
//   * LW with a NEGATIVE (sign-extended) offset
//   * a branch whose operand comes from a load (stall, then flush)
// ============================================================================
module tb_hazards;

    reg         clk;
    reg         reset;
    wire [31:0] out_data;
    wire [31:0] branch_count;
    wire [31:0] mispredict_count;

    integer errors, cycle_count, stall_cycles, flush_cycles, i;
    reg     halted;
    integer bad_target, bad_addr;

    processor DUT (
        .clk              (clk),
        .reset            (reset),
        .out_data         (out_data),
        .branch_count     (branch_count),
        .mispredict_count (mispredict_count)
    );

    localparam OP_RTYPE = 6'b000000;
    localparam OP_JUMP  = 6'b000010;
    localparam OP_BEQ   = 6'b000100;
    localparam OP_BNE   = 6'b000101;
    localparam OP_ORI   = 6'b001101;
    localparam OP_LW    = 6'b100011;
    localparam OP_SW    = 6'b101011;
    localparam NOP      = 32'h0000_0000;

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // Reset held for two cycles; the program is overwritten inside that window
    // (after the first posedge, so it cannot race the module's initial block).
    initial begin
        reset = 1'b1;
        @(posedge clk);
        #1 load_program;
        @(posedge clk);
        #1 reset = 1'b0;
    end

    task load_program;
        begin
            for (i = 0; i < 64; i = i + 1)
                DUT.IF_STAGE.imem[i] = NOP;

            //  idx                                                 expected
            DUT.IF_STAGE.imem[0]  = {OP_ORI, 5'd0,  5'd1,  16'd7};      // r1 = 7
            DUT.IF_STAGE.imem[1]  = NOP;
            DUT.IF_STAGE.imem[2]  = NOP;
            DUT.IF_STAGE.imem[3]  = {OP_ORI, 5'd1,  5'd2,  16'd0};      // 3-DEEP RAW: r2 = 7
            DUT.IF_STAGE.imem[4]  = {OP_ORI, 5'd0,  5'd3,  16'd100};    // r3 = 100
            DUT.IF_STAGE.imem[5]  = NOP;
            DUT.IF_STAGE.imem[6]  = {OP_ORI, 5'd3,  5'd4,  16'd0};      // 2 back: r4 = 100
            DUT.IF_STAGE.imem[7]  = {OP_ORI, 5'd4,  5'd5,  16'd0};      // 1 back: r5 = 100
            DUT.IF_STAGE.imem[8]  = {OP_SW,  5'd0,  5'd5,  16'd40};     // mem[40] = 100
            DUT.IF_STAGE.imem[9]  = {OP_LW,  5'd0,  5'd6,  16'd40};     // r6 = 100
            DUT.IF_STAGE.imem[10] = {OP_ORI, 5'd6,  5'd11, 16'd0};      // STALL -> r11 = 100
            DUT.IF_STAGE.imem[11] = {OP_ORI, 5'd6,  5'd12, 16'd0};      // r12 = 100 (3rd fwd src)
            DUT.IF_STAGE.imem[12] = {OP_LW,  5'd0,  5'd13, 16'd40};     // r13 = 100
            DUT.IF_STAGE.imem[13] = {OP_LW,  5'd0,  5'd14, 16'd40};     // r14 = 100, no stall
            DUT.IF_STAGE.imem[14] = {OP_SW,  5'd0,  5'd14, 16'd44};     // STALL (rt) mem[44]=100
            DUT.IF_STAGE.imem[15] = {OP_ORI, 5'd0,  5'd15, 16'd44};     // r15 = 44
            DUT.IF_STAGE.imem[16] = {OP_LW,  5'd15, 5'd16, 16'd0};      // fwd base -> r16 = 100
            DUT.IF_STAGE.imem[17] = {OP_ORI, 5'd0,  5'd17, 16'd1};      // r17 = 1
            DUT.IF_STAGE.imem[18] = {OP_BEQ, 5'd17, 5'd0,  16'd5};      // NOT taken
            DUT.IF_STAGE.imem[19] = {OP_BNE, 5'd17, 5'd0,  16'd2};      // TAKEN -> idx 22
            DUT.IF_STAGE.imem[20] = {OP_ORI, 5'd0,  5'd18, 16'hDEAD};   // KILLED
            DUT.IF_STAGE.imem[21] = {OP_ORI, 5'd0,  5'd19, 16'hBEEF};   // KILLED
            DUT.IF_STAGE.imem[22] = {OP_BEQ, 5'd17, 5'd17, 16'd2};      // TAKEN -> idx 25
            DUT.IF_STAGE.imem[23] = {OP_LW,  5'd0,  5'd20, 16'd40};     // KILLED wrong-path LOAD
            DUT.IF_STAGE.imem[24] = {OP_SW,  5'd0,  5'd17, 16'd48};     // KILLED wrong-path STORE
            DUT.IF_STAGE.imem[25] = {OP_LW,  5'd0,  5'd21, 16'd40};     // target is a load
            DUT.IF_STAGE.imem[26] = {OP_BEQ, 5'd21, 5'd6,  16'd2};      // STALL then TAKEN -> 29
            // A wrong-path JUMP: it sits in ID exactly when idx 26 resolves
            // taken, so a taken branch must outrank it. If the priority is
            // wrong the jump wins and the poison at idx 50 writes r22.
            DUT.IF_STAGE.imem[27] = {OP_JUMP, 26'd50};                  // KILLED (wrong-path jump)
            DUT.IF_STAGE.imem[28] = {OP_ORI, 5'd0,  5'd23, 16'h2222};   // KILLED
            DUT.IF_STAGE.imem[29] = {OP_ORI, 5'd0,  5'd24, 16'd60};     // r24 = 60
            DUT.IF_STAGE.imem[30] = {OP_SW,  5'd24, 5'd24, 16'd0};      // mem[60] = 60
            DUT.IF_STAGE.imem[31] = {OP_ORI, 5'd0,  5'd25, 16'd64};     // r25 = 64
            DUT.IF_STAGE.imem[32] = {OP_LW,  5'd25, 5'd26, 16'hFFFC};   // NEG offset -4 -> 60
            DUT.IF_STAGE.imem[33] = {OP_ORI, 5'd26, 5'd27, 16'd0};      // STALL -> r27 = 60
            DUT.IF_STAGE.imem[34] = {OP_BNE, 5'd26, 5'd24, 16'd2};      // equal -> NOT taken
            DUT.IF_STAGE.imem[35] = {OP_ORI, 5'd0,  5'd28, 16'd5};      // r28 = 5
            DUT.IF_STAGE.imem[36] = {OP_JUMP, 26'd38};                  // skip idx 37
            DUT.IF_STAGE.imem[37] = {OP_ORI, 5'd0,  5'd29, 16'h3333};   // MUST NOT EXECUTE
            DUT.IF_STAGE.imem[38] = {OP_ORI, 5'd0,  5'd30, 16'd9};      // r30 = 9
            DUT.IF_STAGE.imem[39] = {OP_JUMP, 26'd39};                  // halt

            // Poison: only reachable if the wrong-path jump at idx 27 is
            // wrongly allowed to redirect the PC.
            DUT.IF_STAGE.imem[50] = {OP_ORI, 5'd0,  5'd22, 16'h1111};
        end
    endtask

    initial begin
        errors = 0; cycle_count = 0; stall_cycles = 0; flush_cycles = 0; halted = 1'b0;
        bad_target = 0; bad_addr = 0;
    end

    wire halt_now = DUT.cu_jump && !DUT.flush &&
                    (DUT.jump_target == (DUT.if_id_pc4 - 32'd4));

    always @(posedge clk) begin
        if (!reset) begin
            cycle_count = cycle_count + 1;
            if (DUT.stall) stall_cycles = stall_cycles + 1;
            if (DUT.flush) flush_cycles = flush_cycles + 1;
            if (!halted) begin
                if (DUT.stall)
                    $display("cyc %3d  STALL  (load dest r%0d)", cycle_count, DUT.ex_write_reg);
                if (DUT.flush)
                    $display("cyc %3d  FLUSH  -> PC 0x%08h", cycle_count, DUT.ex_branch_target);
                if (DUT.ex_mem_mem_write)
                    $display("cyc %3d  MEM WR mem[%0d] <= %0d",
                             cycle_count, DUT.ex_mem_alu_result, DUT.ex_mem_rt_data_fwd);
                if (DUT.ex_mem_mem_read)
                    $display("cyc %3d  MEM RD mem[%0d] => %0d",
                             cycle_count, DUT.ex_mem_alu_result, DUT.mem_read_data);
                if (halt_now) begin halted = 1'b1; $display("cyc %3d  HALT", cycle_count); end
            end
        end
    end


    // ---------------------------------------------------------- assertions
    // Direct checks on ADDRESS ARITHMETIC. These exist because both memories
    // wrap at a power of two that divides 2^16, so a wrongly zero-extended
    // negative offset silently aliases onto the correct location and would
    // otherwise go unnoticed. Checking the full 32-bit computed value catches
    // it: with correct sign extension every branch target stays inside the
    // 256-byte program and every data address inside the 1 KB data memory.
    always @(posedge clk) begin
        if (!reset) begin
            if (DUT.id_ex_branch && (DUT.ex_branch_target > 32'd255)) begin
                $display("cyc %3d  ASSERT FAIL  branch target 0x%08h is outside the program (sign-extension bug?)",
                         cycle_count, DUT.ex_branch_target);
                bad_target = bad_target + 1;
            end
            if ((DUT.ex_mem_mem_read || DUT.ex_mem_mem_write) &&
                (DUT.ex_mem_alu_result > 32'd1023)) begin
                $display("cyc %3d  ASSERT FAIL  data address 0x%08h is outside data memory (sign-extension bug?)",
                         cycle_count, DUT.ex_mem_alu_result);
                bad_addr = bad_addr + 1;
            end
        end
    end

    task check_reg;
        input [4:0] num; input [31:0] expected; input [80*8:1] note;
        begin
            if (DUT.RF.regs[num] !== expected) begin
                $display("  FAIL  r%0d = 0x%08h, expected 0x%08h   (%0s)",
                         num, DUT.RF.regs[num], expected, note);
                errors = errors + 1;
            end else
                $display("  pass  r%-2d = %-10d  (%0s)", num, DUT.RF.regs[num], note);
        end
    endtask

    task check_mem;
        input integer byte_addr; input [31:0] expected; input [80*8:1] note;
        begin
            if (DUT.DMEM.mem[byte_addr >> 2] !== expected) begin
                $display("  FAIL  mem[%0d] = 0x%08h, expected 0x%08h   (%0s)",
                         byte_addr, DUT.DMEM.mem[byte_addr >> 2], expected, note);
                errors = errors + 1;
            end else
                $display("  pass  mem[%-3d] = %-10d  (%0s)",
                         byte_addr, DUT.DMEM.mem[byte_addr >> 2], note);
        end
    endtask

    task check_val;
        input [31:0] actual; input [31:0] expected; input [80*8:1] note;
        begin
            if (actual !== expected) begin
                $display("  FAIL  %0s = %0d, expected %0d", note, actual, expected);
                errors = errors + 1;
            end else
                $display("  pass  %0s = %0d", note, actual);
        end
    endtask

    initial begin
        wait (reset === 1'b0);
        wait (halted === 1'b1);
        repeat (8) @(posedge clk);
        #1;

        $display("\n==============================================================");
        $display(" HAZARD / FLUSH CORNER CASES");
        $display("==============================================================");
        check_reg(5'd1,  32'd7,   "ori r1 = 7");
        check_reg(5'd2,  32'd7,   "RAW 3 INSTRUCTIONS APART (needs 3rd fwd source)");
        check_reg(5'd4,  32'd100, "RAW 2 apart");
        check_reg(5'd5,  32'd100, "RAW 1 apart");
        check_reg(5'd6,  32'd100, "lw after sw, same address");
        check_reg(5'd11, 32'd100, "load value 1 apart  [STALL]");
        check_reg(5'd12, 32'd100, "load value 2 apart, right after a stall");
        check_reg(5'd13, 32'd100, "back-to-back load #1");
        check_reg(5'd14, 32'd100, "back-to-back load #2 (must not stall)");
        check_reg(5'd16, 32'd100, "lw with forwarded base register");
        check_reg(5'd20, 32'd0,   "WRONG-PATH LOAD must not write");
        check_reg(5'd18, 32'd0,   "KILLED after taken bne");
        check_reg(5'd19, 32'd0,   "KILLED after taken bne");
        check_reg(5'd21, 32'd100, "branch target is itself a load");
        check_reg(5'd22, 32'd0,   "wrong-path JUMP must lose to the taken branch");
        check_reg(5'd23, 32'd0,   "KILLED after stall+flush");
        check_reg(5'd26, 32'd60,  "lw with NEGATIVE offset -4(r25)");
        check_reg(5'd27, 32'd60,  "consumer of negative-offset load [STALL]");
        check_reg(5'd28, 32'd5,   "bne not taken -> fall through");
        check_reg(5'd29, 32'd0,   "jump skipped this instruction");
        check_reg(5'd30, 32'd9,   "jump target reached");

        $display("");
        check_mem(40,  32'd100, "sw with forwarded store data");
        check_mem(44,  32'd100, "sw whose data came from a stalled load");
        check_mem(60,  32'd60,  "sw with base AND data both forwarded (r24)");
        check_mem(48,  32'd0,   "WRONG-PATH STORE must not commit");

        $display("");
        check_val(branch_count,     32'd5, "branches resolved");
        check_val(mispredict_count, 32'd3, "mispredictions");
        check_val(flush_cycles,     32'd3, "flush events");
        check_val(bad_target,       32'd0, "branch targets out of range");
        check_val(bad_addr,         32'd0, "data addresses out of range");
        check_val(stall_cycles,     32'd4, "load-use stall cycles");

        $display("\n==============================================================");
        if (errors == 0) $display(" RESULT: ALL CORNER-CASE CHECKS PASSED");
        else             $display(" RESULT: %0d CHECK(S) FAILED", errors);
        $display("==============================================================\n");
        $finish;
    end

    initial begin
        #20000;
        $display("ERROR: timeout, halt never reached");
        $finish;
    end

endmodule
