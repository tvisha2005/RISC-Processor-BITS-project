`timescale 1ns / 1ps

// ============================================================================
// TOP-LEVEL 5-STAGE PIPELINED PROCESSOR
//
// Stages: IF -> ID -> EX -> MEM -> WB
//
// What changed in this revision
// -----------------------------
// 1. MEM stage is real. data_memory is instantiated and driven by
//    ex_mem_mem_read / ex_mem_mem_write / ex_mem_alu_result, with the store
//    data taken from ex_mem_rt_data_fwd (post-forwarding rt).
//
// 2. Load-use hazard: hazard_unit asserts stall, which freezes PC and IF/ID
//    and injects one bubble into ID/EX.
//
// 3. BEQ / BNE, resolved in EX with a static predict-not-taken policy.
//    Resolving in EX (rather than ID) means the comparison operands come out
//    of the existing forwarding network, so no extra forwarding paths or extra
//    stall cases are needed. Cost: two wrong-path instructions to discard.
//
//    Timing of a taken branch (branch is in EX during cycle T):
//      cycle T    : branch in EX, outcome known, flush asserted
//                   IF/ID holds branch+1, and branch+2 is being fetched
//      posedge T+1: ID/EX <= bubble        (kills branch+1)
//                   IF/ID <= NOP          (kills branch+2)
//                   PC    <= branch target
//      posedge T+2: IF/ID <= target instruction
//    Nothing on the wrong path ever reaches MEM, so no speculative store can
//    corrupt memory and no speculative write can reach the register file.
//
// 4. Third forwarding source (see forwarding_unit.v) closing the 3-instruction
//    RAW gap created by this pipeline's writeback/read timing.
// ============================================================================
module processor (
    input             clk,
    input             reset,
    output     [31:0] out_data,
    // Observability outputs (for the testbench; harmless if left unconnected)
    output     [31:0] branch_count,
    output     [31:0] mispredict_count
);
    localparam NOP = 32'h0000_0000;

    // ======================= IF/ID pipeline registers =======================
    wire [31:0] if_id_instr;
    wire [31:0] if_id_pc4;

    // ============================== ID STAGE ================================
    wire [5:0]  id_opcode = if_id_instr[31:26];
    wire [4:0]  id_rs     = if_id_instr[25:21];
    wire [4:0]  id_rt     = if_id_instr[20:16];
    wire [4:0]  id_rd     = if_id_instr[15:11];
    wire [5:0]  id_funct  = if_id_instr[5:0];
    wire [15:0] id_imm16  = if_id_instr[15:0];

    // Control unit signals
    wire        cu_reg_write, cu_reg_dst, cu_alu_src;
    wire        cu_mem_read,  cu_mem_write, cu_mem_to_reg, cu_jump;
    wire        cu_branch,    cu_branch_ne, cu_sign_ext;
    wire        cu_uses_rs,   cu_uses_rt;
    wire [2:0]  cu_alu_op;

    // Immediate: sign-extended for LW/SW/BEQ/BNE, zero-extended for ORI/LUI
    wire [31:0] id_imm_ext = cu_sign_ext ? {{16{id_imm16[15]}}, id_imm16}
                                         : {16'd0,             id_imm16};

    // Unconditional jump is still resolved in ID
    wire [31:0] jump_target = {if_id_pc4[31:28], if_id_instr[25:0], 2'b00};

    // Register file read data
    wire [31:0] rf_rs_data, rf_rt_data;

    // WB stage write-back data
    wire [31:0] wb_write_data;

    // ======================= ID/EX pipeline registers =======================
    reg        id_ex_reg_write, id_ex_reg_dst, id_ex_alu_src;
    reg        id_ex_mem_read,  id_ex_mem_write, id_ex_mem_to_reg;
    reg        id_ex_branch,    id_ex_branch_ne;          // NEW
    reg [2:0]  id_ex_alu_op;
    reg [31:0] id_ex_rs_data,   id_ex_rt_data;
    reg [4:0]  id_ex_rs,        id_ex_rt, id_ex_rd;
    reg [31:0] id_ex_imm_ext;                             // renamed (was zero_ext only)
    reg [31:0] id_ex_pc4;                                 // NEW: branch base address

    // ====================== EX/MEM pipeline registers =======================
    reg        ex_mem_reg_write, ex_mem_mem_read, ex_mem_mem_write, ex_mem_mem_to_reg;
    reg [31:0] ex_mem_alu_result;
    reg [31:0] ex_mem_rt_data_fwd;   // store data
    reg [4:0]  ex_mem_write_reg;

    // ====================== MEM/WB pipeline registers =======================
    reg        mem_wb_reg_write, mem_wb_mem_to_reg;
    reg [31:0] mem_wb_alu_result;
    reg [31:0] mem_wb_mem_data;
    reg [4:0]  mem_wb_write_reg;

    // ============ Register-file write-port shadow (3rd fwd source) ===========
    // Holds the write that was committed to the register file at the most
    // recent clock edge, i.e. the producer 3 instructions ahead of whatever is
    // in EX right now.
    reg        rf_wr_reg_write;
    reg [4:0]  rf_wr_write_reg;
    reg [31:0] rf_wr_write_data;

    // ============================== EX STAGE ================================
    wire [1:0]  forward_a, forward_b;

    // Operand A (rs), newest source wins
    wire [31:0] alu_a =
        (forward_a == 2'b10) ? ex_mem_alu_result :
        (forward_a == 2'b01) ? wb_write_data     :
        (forward_a == 2'b11) ? rf_wr_write_data  :
                               id_ex_rs_data;

    // Operand B (rt) before the ALUSrc mux. This is also the store data and
    // the second branch-comparison operand.
    wire [31:0] alu_b_fwd =
        (forward_b == 2'b10) ? ex_mem_alu_result :
        (forward_b == 2'b01) ? wb_write_data     :
        (forward_b == 2'b11) ? rf_wr_write_data  :
                               id_ex_rt_data;

    // ALUSrc
    wire [31:0] alu_b = id_ex_alu_src ? id_ex_imm_ext : alu_b_fwd;

    wire [31:0] alu_result;
    wire        alu_zero;

    // RegDst
    wire [4:0]  ex_write_reg = id_ex_reg_dst ? id_ex_rd : id_ex_rt;

    // -------------------- branch resolution (EX) --------------------
    wire        predict_taken;                       // static 0 = not taken
    wire        ex_branch_cond  = id_ex_branch_ne ? ~alu_zero : alu_zero;
    wire        ex_branch_taken = id_ex_branch & ex_branch_cond;
    wire [31:0] ex_branch_target = id_ex_pc4 + (id_ex_imm_ext << 2);

    // Mispredict: the resolved outcome disagrees with what was predicted.
    // With predict_taken tied low this reduces to "the branch was taken", but
    // it is written as a comparison so a dynamic predictor drops straight in.
    wire        flush = id_ex_branch & (ex_branch_taken != predict_taken);

    // ------------------------- pipeline control -------------------------
    wire stall;

    // A taken branch outranks a jump sitting in ID: that jump is wrong-path.
    wire        redirect    = flush | cu_jump;
    wire [31:0] redirect_pc = flush ? ex_branch_target : jump_target;

    // ID/EX gets a bubble when we flush the wrong path, when a jump retires in
    // ID (J has nothing to do in EX), or when a load-use stall is inserted.
    wire        bubble_id_ex = flush | cu_jump | stall;

    // ============================= MEM STAGE ================================
    wire [31:0] mem_read_data;

    // ============================== WB STAGE ================================
    assign wb_write_data = mem_wb_mem_to_reg ? mem_wb_mem_data : mem_wb_alu_result;

    // ============================ SUBMODULES ================================
    instruction_fetch IF_STAGE (
        .clk         (clk),
        .reset       (reset),
        .stall       (stall),
        .redirect    (redirect),
        .redirect_pc (redirect_pc),
        .if_id_instr (if_id_instr),
        .if_id_pc4   (if_id_pc4)
    );

    control_unit CU (
        .opcode     (id_opcode),
        .funct      (id_funct),
        .reg_write  (cu_reg_write),
        .reg_dst    (cu_reg_dst),
        .alu_src    (cu_alu_src),
        .alu_op     (cu_alu_op),
        .mem_read   (cu_mem_read),
        .mem_write  (cu_mem_write),
        .mem_to_reg (cu_mem_to_reg),
        .jump       (cu_jump),
        .branch     (cu_branch),
        .branch_ne  (cu_branch_ne),
        .sign_ext   (cu_sign_ext),
        .uses_rs    (cu_uses_rs),
        .uses_rt    (cu_uses_rt)
    );

    register_file RF (
        .clk        (clk),
        .reset      (reset),
        .rs         (id_rs),
        .rt         (id_rt),
        .rs_data    (rf_rs_data),
        .rt_data    (rf_rt_data),
        .rd         (mem_wb_write_reg),
        .write_data (wb_write_data),
        .reg_write  (mem_wb_reg_write)
    );

    hazard_unit HAZ (
        .id_ex_mem_read  (id_ex_mem_read),
        .id_ex_write_reg (ex_write_reg),
        .id_rs           (id_rs),
        .id_rt           (id_rt),
        .id_uses_rs      (cu_uses_rs),
        .id_uses_rt      (cu_uses_rt),
        .stall           (stall)
    );

    forwarding_unit FWD (
        .id_ex_rs         (id_ex_rs),
        .id_ex_rt         (id_ex_rt),
        .ex_mem_rd        (ex_mem_write_reg),
        .ex_mem_reg_write (ex_mem_reg_write),
        .mem_wb_rd        (mem_wb_write_reg),
        .mem_wb_reg_write (mem_wb_reg_write),
        .rf_wr_rd         (rf_wr_write_reg),
        .rf_wr_reg_write  (rf_wr_reg_write),
        .forward_a        (forward_a),
        .forward_b        (forward_b)
    );

    alu ALU (
        .a      (alu_a),
        .b      (alu_b),
        .alu_op (id_ex_alu_op),
        .result (alu_result),
        .zero   (alu_zero)
    );

    branch_predictor BP (
        .clk              (clk),
        .reset            (reset),
        .ex_branch_valid  (id_ex_branch),
        .ex_branch_taken  (ex_branch_taken),
        .predict_taken    (predict_taken),
        .branch_count     (branch_count),
        .mispredict_count (mispredict_count)
    );

    data_memory DMEM (
        .clk        (clk),
        .mem_read   (ex_mem_mem_read),
        .mem_write  (ex_mem_mem_write),
        .addr       (ex_mem_alu_result),
        .write_data (ex_mem_rt_data_fwd),
        .read_data  (mem_read_data)
    );

    // ========================== ID/EX REGISTER ==============================
    always @(posedge clk or posedge reset) begin
        if (reset || bubble_id_ex) begin
            id_ex_reg_write  <= 1'b0;
            id_ex_reg_dst    <= 1'b0;
            id_ex_alu_src    <= 1'b0;
            id_ex_mem_read   <= 1'b0;
            id_ex_mem_write  <= 1'b0;
            id_ex_mem_to_reg <= 1'b0;
            id_ex_branch     <= 1'b0;
            id_ex_branch_ne  <= 1'b0;
            id_ex_alu_op     <= 3'd0;
            id_ex_rs_data    <= 32'd0;
            id_ex_rt_data    <= 32'd0;
            id_ex_rs         <= 5'd0;
            id_ex_rt         <= 5'd0;
            id_ex_rd         <= 5'd0;
            id_ex_imm_ext    <= 32'd0;
            id_ex_pc4        <= 32'd0;
        end else begin
            id_ex_reg_write  <= cu_reg_write;
            id_ex_reg_dst    <= cu_reg_dst;
            id_ex_alu_src    <= cu_alu_src;
            id_ex_mem_read   <= cu_mem_read;
            id_ex_mem_write  <= cu_mem_write;
            id_ex_mem_to_reg <= cu_mem_to_reg;
            id_ex_branch     <= cu_branch;
            id_ex_branch_ne  <= cu_branch_ne;
            id_ex_alu_op     <= cu_alu_op;
            id_ex_rs_data    <= rf_rs_data;
            id_ex_rt_data    <= rf_rt_data;
            id_ex_rs         <= id_rs;
            id_ex_rt         <= id_rt;
            id_ex_rd         <= id_rd;
            id_ex_imm_ext    <= id_imm_ext;
            id_ex_pc4        <= if_id_pc4;
        end
    end

    // ========================= EX/MEM REGISTER ==============================
    // No stall or flush case here: once an instruction reaches EX it is on the
    // correct path (wrong-path instructions are bubbled at ID/EX) and nothing
    // downstream of EX can stall.
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ex_mem_reg_write   <= 1'b0;
            ex_mem_mem_read    <= 1'b0;
            ex_mem_mem_write   <= 1'b0;
            ex_mem_mem_to_reg  <= 1'b0;
            ex_mem_alu_result  <= 32'd0;
            ex_mem_rt_data_fwd <= 32'd0;
            ex_mem_write_reg   <= 5'd0;
        end else begin
            ex_mem_reg_write   <= id_ex_reg_write;
            ex_mem_mem_read    <= id_ex_mem_read;
            ex_mem_mem_write   <= id_ex_mem_write;
            ex_mem_mem_to_reg  <= id_ex_mem_to_reg;
            ex_mem_alu_result  <= alu_result;      // LW/SW: effective address
            ex_mem_rt_data_fwd <= alu_b_fwd;       // SW: store data
            ex_mem_write_reg   <= ex_write_reg;
        end
    end

    // ========================= MEM/WB REGISTER ==============================
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_wb_reg_write  <= 1'b0;
            mem_wb_mem_to_reg <= 1'b0;
            mem_wb_alu_result <= 32'd0;
            mem_wb_mem_data   <= 32'd0;
            mem_wb_write_reg  <= 5'd0;
        end else begin
            mem_wb_reg_write  <= ex_mem_reg_write;
            mem_wb_mem_to_reg <= ex_mem_mem_to_reg;
            mem_wb_alu_result <= ex_mem_alu_result;
            mem_wb_mem_data   <= mem_read_data;
            mem_wb_write_reg  <= ex_mem_write_reg;
        end
    end

    // ================= REGISTER-FILE WRITE-PORT SHADOW ======================
    // Mirrors exactly what the register file latched on this same edge.
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            rf_wr_reg_write  <= 1'b0;
            rf_wr_write_reg  <= 5'd0;
            rf_wr_write_data <= 32'd0;
        end else begin
            rf_wr_reg_write  <= mem_wb_reg_write;
            rf_wr_write_reg  <= mem_wb_write_reg;
            rf_wr_write_data <= wb_write_data;
        end
    end

    assign out_data = wb_write_data;

endmodule
