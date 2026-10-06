`timescale 1ns / 1ps

// ============================================================================
// BRANCH PREDICTOR  --  static "predict not taken"
//
// The prediction is simply "fall through", which costs nothing in the fetch
// path: the PC keeps incrementing and the instructions after the branch are
// fetched speculatively. The branch is resolved in EX; if the real outcome
// disagrees with the prediction, processor.v raises a flush and redirects.
//
// The counters are for observability only (the testbench reads them) and do
// not affect the datapath.
//
// UPGRADE PATH -- to make this dynamic, replace the constant with a table and
// route a prediction back into the fetch stage:
//
//   reg [1:0] bht [0:63];                        // 2-bit saturating counters
//   assign predict_taken = bht[if_pc[7:2]][1];
//   ... on resolution, increment/decrement bht[ex_pc[7:2]]
//
// The rest of the pipeline already compares the actual outcome against
// predict_taken rather than against a hard-coded 0, so only this module and
// the redirect target mux (which would then also need a "predicted taken but
// actually not taken" recovery path back to pc+4) would change.
// ============================================================================
module branch_predictor (
    input             clk,
    input             reset,
    input             ex_branch_valid,   // a real (non-bubble) branch is in EX
    input             ex_branch_taken,   // its resolved outcome
    output            predict_taken,     // static: never taken
    output reg [31:0] branch_count,
    output reg [31:0] mispredict_count
);

    assign predict_taken = 1'b0;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            branch_count     <= 32'd0;
            mispredict_count <= 32'd0;
        end else if (ex_branch_valid) begin
            branch_count <= branch_count + 32'd1;
            if (ex_branch_taken != predict_taken)
                mispredict_count <= mispredict_count + 32'd1;
        end
    end

endmodule
