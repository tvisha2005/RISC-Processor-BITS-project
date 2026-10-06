`timescale 1ns / 1ps

// ============================================================================
// MAIN SELF-CHECKING TESTBENCH
//
// Runs the program held in instr_fetch.v, which exercises the original
// instruction set plus the newly added LW, SW, BEQ and BNE.
//
// Checked after the program parks itself on its halt instruction:
//   * every architectural register against a hand-computed expected value
//   * every touched data-memory word, INCLUDING words that must still be zero
//     because the store that would have written them sat on a wrong path
//   * branch / misprediction counters, flush events and stall cycles
//
// A per-cycle trace of stalls, flushes, jumps and memory traffic is printed so
// the hazard and flush behaviour can be inspected directly.
// ============================================================================
module tb_processor;

    reg         clk;
    reg         reset;
    wire [31:0] out_data;
    wire [31:0] branch_count;
    wire [31:0] mispredict_count;

    integer errors;
    integer cycle_count;
    integer stall_cycles;
    integer flush_cycles;
    reg     halted;
    integer bad_target, bad_addr;

    processor DUT (
        .clk              (clk),
        .reset            (reset),
        .out_data         (out_data),
        .branch_count     (branch_count),
        .mispredict_count (mispredict_count)
    );

    // ------------------------------------------------------------------ clock
    initial clk = 1'b0;
    always #5 clk = ~clk;                  // 10 ns period

    // ------------------------------------------------------------------ reset
    // Released just after a posedge so deassertion never coincides with the
    // register file's negedge read.
    initial begin
        reset = 1'b1;
        repeat (2) @(posedge clk);
        #1 reset = 1'b0;
    end

    initial begin
        cycle_count  = 0;
        stall_cycles = 0;
        flush_cycles = 0;
        errors       = 0;
        halted       = 1'b0;
        bad_target   = 0;
        bad_addr     = 0;
    end

    initial begin
        $dumpfile("processor.vcd");
        $dumpvars(0, tb_processor);
    end

    // The program ends with a jump to itself; that is the halt condition.
    wire halt_now = DUT.cu_jump && !DUT.flush &&
                    (DUT.jump_target == (DUT.if_id_pc4 - 32'd4));

    // -------------------------------------------------- counters + trace
    // Deliberately ONE always block: counting and printing in separate blocks
    // would race and print inconsistent cycle numbers.
    always @(posedge clk) begin
        if (!reset) begin
            cycle_count = cycle_count + 1;
            if (DUT.stall) stall_cycles = stall_cycles + 1;
            if (DUT.flush) flush_cycles = flush_cycles + 1;

            if (!halted) begin
                if (DUT.stall)
                    $display("cyc %3d  STALL   load-use: instruction in ID reads r%0d, still in the load's EX stage",
                             cycle_count, DUT.ex_write_reg);
                if (DUT.flush)
                    $display("cyc %3d  FLUSH   branch TAKEN (mispredicted) -> kill 2 wrong-path instrs, PC <= 0x%08h",
                             cycle_count, DUT.ex_branch_target);
                if (DUT.id_ex_branch && !DUT.ex_branch_taken)
                    $display("cyc %3d  BRANCH  not taken (prediction correct, no penalty)",
                             cycle_count);
                if (DUT.cu_jump && !DUT.flush && !halt_now)
                    $display("cyc %3d  JUMP    PC <= 0x%08h", cycle_count, DUT.jump_target);
                if (DUT.ex_mem_mem_write)
                    $display("cyc %3d  MEM WR  mem[%0d] <= 0x%08h",
                             cycle_count, DUT.ex_mem_alu_result, DUT.ex_mem_rt_data_fwd);
                if (DUT.ex_mem_mem_read)
                    $display("cyc %3d  MEM RD  mem[%0d] => 0x%08h",
                             cycle_count, DUT.ex_mem_alu_result, DUT.mem_read_data);
                if (halt_now) begin
                    halted = 1'b1;
                    $display("cyc %3d  HALT    reached jump-to-self", cycle_count);
                end
            end
        end
    end

    // ------------------------------------------------------------------ tasks

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
        input [4:0]     num;
        input [31:0]    expected;
        input [80*8:1]  note;
        begin
            if (DUT.RF.regs[num] !== expected) begin
                $display("  FAIL  r%0d = 0x%08h, expected 0x%08h   (%0s)",
                         num, DUT.RF.regs[num], expected, note);
                errors = errors + 1;
            end else
                $display("  pass  r%-2d = 0x%08h   (%0s)", num, DUT.RF.regs[num], note);
        end
    endtask

    task check_mem;
        input integer   byte_addr;
        input [31:0]    expected;
        input [80*8:1]  note;
        begin
            if (DUT.DMEM.mem[byte_addr >> 2] !== expected) begin
                $display("  FAIL  mem[%0d] = 0x%08h, expected 0x%08h   (%0s)",
                         byte_addr, DUT.DMEM.mem[byte_addr >> 2], expected, note);
                errors = errors + 1;
            end else
                $display("  pass  mem[%-3d] = 0x%08h   (%0s)",
                         byte_addr, DUT.DMEM.mem[byte_addr >> 2], note);
        end
    endtask

    task check_val;
        input [31:0]    actual;
        input [31:0]    expected;
        input [80*8:1]  note;
        begin
            if (actual !== expected) begin
                $display("  FAIL  %0s = %0d, expected %0d", note, actual, expected);
                errors = errors + 1;
            end else
                $display("  pass  %0s = %0d", note, actual);
        end
    endtask

    // ------------------------------------------------------------- main check
    initial begin
        wait (reset === 1'b0);

        // Run until the halt instruction, then let the pipeline drain.
        wait (halted === 1'b1);
        repeat (8) @(posedge clk);
        #1;

        $display("\n==============================================================");
        $display(" ORIGINAL INSTRUCTIONS (regression)");
        $display("==============================================================");
        check_reg(5'd3,  32'd128,      "lsl  r3 = 16<<3");
        check_reg(5'd6,  32'd1,        "ori  r6 = 1 (loop setup, last writer of r6)");
        check_reg(5'd10, 32'd1,        "untouched reset value");

        $display("\n LOAD / STORE + LOAD-USE STALLS");
        check_reg(5'd11, 32'h00001234, "ori  r11 = 0x1234");
        check_reg(5'd12, 32'h00001234, "lw   r12 = mem[0] (load right after store)");
        check_reg(5'd13, 32'h00001234, "ori  r13 = r12       [LOAD-USE STALL]");
        check_reg(5'd14, 32'h00001234, "lw   r14 = mem[4]");
        check_reg(5'd15, 32'h00001234, "lw   r15 = mem[8]");
        check_reg(5'd4,  32'hCAFEBABE, "lw   r4  = mem[252] (pre-loaded word)");
        check_reg(5'd1,  32'hCAFEBABE, "ori  r1  = r4        [LOAD-USE STALL]");
        check_reg(5'd8,  32'd16,       "lw   r8  = mem[24]");
        check_reg(5'd9,  32'd1,        "slt  r9  = (16<128)  [LOAD-USE STALL]");

        $display("\n BRANCH NOT TAKEN (prediction correct, no flush)");
        check_reg(5'd19, 32'd1,        "ori  r19 runs after beq not taken");
        check_reg(5'd20, 32'd2,        "ori  r20 runs after bne not taken");

        $display("\n BRANCH TAKEN (mispredict: wrong path must be killed)");
        check_reg(5'd23, 32'd7,        "beq target reached");
        check_reg(5'd25, 32'd8,        "bne target reached");
        check_reg(5'd29, 32'd9,        "target reached after stall + flush");
        check_reg(5'd30, 32'd9,        "ori  r30 = 9");
        check_reg(5'd2,  32'h00000ABC, "target reached after fully forwarded beq");
        check_reg(5'd21, 32'd0,        "KILLED wrong-path instr (idx 24)");
        check_reg(5'd22, 32'd0,        "SKIPPED instr (idx 26)");
        check_reg(5'd24, 32'd0,        "KILLED wrong-path instr (idx 29)");
        check_reg(5'd27, 32'd0,        "KILLED wrong-path instr (idx 34)");
        check_reg(5'd28, 32'd0,        "KILLED wrong-path instr (idx 35)");
        check_reg(5'd31, 32'd0,        "KILLED wrong-path instr (idx 40)");

        $display("\n BACKWARD BRANCH LOOP (negative sign-extended offset, 4 passes)");
        check_reg(5'd5,  32'd0,        "loop counter drained 8 -> 0");
        check_reg(5'd7,  32'd16,       "iteration marker 1<<4 = 16");

        $display("\n DATA MEMORY");
        check_mem(0,   32'h00001234, "sw from r11");
        check_mem(4,   32'h00001234, "sw from r13");
        check_mem(8,   32'h00001234, "sw from r14 (store that was stalled)");
        check_mem(24,  32'd16,       "sw from r7 after the loop");
        check_mem(28,  32'd1,        "sw from r9");
        check_mem(252, 32'hCAFEBABE, "pre-loaded word, untouched");
        check_mem(12,  32'd0,        "WRONG-PATH STORE must not commit (idx 25)");
        check_mem(16,  32'd0,        "WRONG-PATH STORE must not commit (idx 30)");
        check_mem(20,  32'd0,        "WRONG-PATH STORE must not commit (idx 39)");

        $display("\n BRANCH PREDICTOR STATISTICS");
        // Branches executed: idx 19 and 21 not taken; idx 23, 28, 33, 38 taken;
        // idx 47 four times (taken, taken, taken, not taken).
        // Total 10, of which 7 taken. Static policy is "not taken", so every
        // taken branch counts as a misprediction and costs one flush.
        check_val(branch_count,     32'd10, "branches resolved");
        check_val(mispredict_count, 32'd7,  "mispredictions (taken branches)");
        check_val(flush_cycles,     32'd7,  "flush events");
        check_val(bad_target,       32'd0, "branch targets out of range");
        check_val(bad_addr,         32'd0, "data addresses out of range");
        check_val(stall_cycles,     32'd5,  "load-use stall cycles");

        $display("\n==============================================================");
        if (errors == 0)
            $display(" RESULT: ALL CHECKS PASSED   (%0d cycles to halt)", cycle_count);
        else
            $display(" RESULT: %0d CHECK(S) FAILED", errors);
        $display("==============================================================\n");
        $finish;
    end

    // Safety net in case the program never halts
    initial begin
        #20000;
        $display("ERROR: timeout, halt never reached");
        $finish;
    end

endmodule
