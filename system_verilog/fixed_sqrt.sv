`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: fixed_sqrt
// Description: Fixed-point square root for Q16.16 using integer Newton method
// Input:  unsigned fixed_t (>= 0)
// Output: sqrt(value) in Q16.16
// Latency: ~20-30 cycles (iterative, done pulses when complete)
//////////////////////////////////////////////////////////////////////////////

module fixed_sqrt import fixed_math_pkg::*; (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  fixed_t      in,         // Q16.16 >= 0
    output fixed_t      out,
    output logic        done
);

    // Internal: operate on 48-bit integers for precision
    // val = in <<< 16  (convert to Q32.32 intermediate)
    // then sqrt, then >>> 16 back to Q16.16

    typedef logic unsigned [47:0] u48_t;

    u48_t val, res, x_new;
    logic running;
    logic [4:0] iter;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running <= 0;
            done <= 0;
            out <= FIXED_ZERO;
        end else begin
            done <= 0;
            if (start && !running) begin
                running <= 1;
                iter <= 0;
                // val = in * 2^16  (Q16.16 -> Q32.32)
                val <= u48_t'($unsigned(in)) <<< 16;
                // Initial guess: x0 = val / 2 + 1 (avoid 0)
                x_new <= (u48_t'($unsigned(in)) <<< 15) + 1;
            end else if (running) begin
                u48_t x = x_new;
                u48_t x_next = (x + (val / x)) >>> 1;
                x_new <= x_next;
                if (iter == 20 || x_next == x) begin
                    running <= 0;
                    done <= 1;
                    // Convert back to Q16.16
                    out <= fixed_t'(x_next >>> 16);
                end else begin
                    iter <= iter + 1;
                end
            end
        end
    end

endmodule
