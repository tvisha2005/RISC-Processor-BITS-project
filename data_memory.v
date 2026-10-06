`timescale 1ns / 1ps

// ============================================================================
// DATA MEMORY  (the MEM pipeline stage)
//
// 256 words = 1 KB. Word addressed internally: the byte address coming from
// the ALU is divided by 4 (addr[9:2]), so LW/SW are word operations and the
// effective address is expected to be 4-byte aligned.
//
// Timing:
//   read  : combinational (asynchronous) during the MEM cycle, so the loaded
//           word is captured into MEM/WB at the end of that same cycle.
//   write : synchronous, at the posedge that ends the MEM cycle.
//
// Because the read is combinational and the write is at the clock edge, a
// store followed immediately by a load of the same address returns the newly
// stored value: the write lands at the edge where the load enters MEM, and
// the load's combinational read then sees the updated array.
// ============================================================================
module data_memory (
    input         clk,
    input         mem_read,
    input         mem_write,
    input  [31:0] addr,        // byte address from the ALU
    input  [31:0] write_data,  // store data (rt, after forwarding)
    output [31:0] read_data
);

    reg [31:0] mem [0:255];
    integer j;

    wire [7:0] index = addr[9:2];   // word index, wraps within 1 KB

    initial begin
        for (j = 0; j < 256; j = j + 1)
            mem[j] = 32'h0000_0000;

        // Pre-loaded constant so that LW can be tested on data that was not
        // just written by this program.  Byte address 252 -> word index 63.
        mem[63] = 32'hCAFE_BABE;
    end

    // Gated so that non-load instructions propagate a clean 0 rather than
    // whatever happens to sit at the ALU-result address.
    assign read_data = mem_read ? mem[index] : 32'd0;

    always @(posedge clk) begin
        if (mem_write)
            mem[index] <= write_data;
    end

endmodule
