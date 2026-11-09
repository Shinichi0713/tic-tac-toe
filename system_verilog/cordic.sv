`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: cordic
// Description: Pipelined CORDIC for atan2, sqrt, sin, cos
//              Uses Q16.16 fixed-point.
//   mode=0: Vectoring -> computes r = sqrt(x^2+y^2) and atan2(y,x)
//   mode=1: Rotation  -> computes (x*cos(z) - y*sin(z), x*sin(z) + y*cos(z))
// Latency = NUM_ITER + 2 cycles
//////////////////////////////////////////////////////////////////////////////

module cordic import fixed_math_pkg::*; (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic        mode,       // 0=vectoring, 1=rotation
    input  fixed_t      x_in,
    input  fixed_t      y_in,
    input  fixed_t      z_in,       // angle for rotation mode
    output fixed_t      x_out,
    output fixed_t      y_out,
    output fixed_t      z_out,      // angle for vectoring mode
    output logic        done
);

    localparam int NUM_ITER = 16;

    // arctan table: atan(2^-i) in Q16.16 [radians]
    // Precomputed values
    const fixed_t atan_table [0:15] = '{
        32'sh0000_C90F, // atan(1)     = 0.785398
        32'sh0000_76B1, // atan(1/2)   = 0.463648
        32'sh0000_3EB6, // atan(1/4)   = 0.244979
        32'sh0000_1FD5, // atan(1/8)   = 0.124355
        32'sh0000_0FFA, // atan(1/16)  = 0.062419
        32'sh0000_07FF, // atan(1/32)  = 0.031240
        32'sh0000_0400, // atan(1/64)  = 0.015623
        32'sh0000_0200, // atan(1/128) = 0.007812
        32'sh0000_0100, // atan(1/256) = 0.003906
        32'sh0000_0080, // atan(1/512) = 0.001953
        32'sh0000_0040, // atan(1/1024)= 0.000977
        32'sh0000_0020, // atan(1/2048)= 0.000488
        32'sh0000_0010, // atan(1/4096)= 0.000244
        32'sh0000_0008, // atan(1/8192)= 0.000122
        32'sh0000_0004, // atan(1/16384)=0.000061
        32'sh0000_0002  // atan(1/32768)=0.000031
    };

    // CORDIC gain compensation ~ 0.607252935 * 2 = 1.21450587 for pre-scaling
    // We ignore gain for atan2 (only care about angle), but for sqrt we must compensate.
    // K ≈ 0.60725 in Q16.16 = 0x0000_9B74
    localparam fixed_t K_GAIN = 32'sh0000_9B74;

    // Pipeline registers
    fixed_t x_pipe [0:NUM_ITER];
    fixed_t y_pipe [0:NUM_ITER];
    fixed_t z_pipe [0:NUM_ITER];
    logic vec_mode_pipe [0:NUM_ITER]; // propagate mode

    logic running;
    logic [$clog2(NUM_ITER+1)-1:0] iter_cnt;

    // -------------------------------------------------------------------------
    // Control
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            running <= 0;
            iter_cnt <= 0;
            done <= 0;
        end else begin
            done <= 0;
            if (start && !running) begin
                running <= 1;
                iter_cnt <= 0;
                // Initialize pipeline stage 0
                x_pipe[0] <= x_in;
                y_pipe[0] <= y_in;
                z_pipe[0] <= z_in;
                vec_mode_pipe[0] <= ~mode; // 1 for vectoring
            end else if (running) begin
                if (iter_cnt == NUM_ITER-1) begin
                    running <= 0;
                    iter_cnt <= 0;
                    done <= 1;
                end else begin
                    iter_cnt <= iter_cnt + 1;
                    // Shift pipeline (done combinationally below)
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // CORDIC iterations (combinational for each stage, registered at next edge)
    // For synthesis, this is a generate block creating NUM_ITER stages.
    // -------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < NUM_ITER; i++) begin : cordic_stage
            always_ff @(posedge clk) begin
                if (running && (iter_cnt == i)) begin
                    fixed_t x_shift, y_shift;
                    // Arithmetic right shift for Q16.16
                    x_shift = x_pipe[i] >>> i;
                    y_shift = y_pipe[i] >>> i;

                    if (vec_mode_pipe[i]) begin
                        // Vectoring mode: rotate to drive y to 0
                        if (y_pipe[i] > 0) begin
                            x_pipe[i+1] <= x_pipe[i] + y_shift;
                            y_pipe[i+1] <= y_pipe[i] - x_shift;
                            z_pipe[i+1] <= z_pipe[i] + atan_table[i];
                        end else begin
                            x_pipe[i+1] <= x_pipe[i] - y_shift;
                            y_pipe[i+1] <= y_pipe[i] + x_shift;
                            z_pipe[i+1] <= z_pipe[i] - atan_table[i];
                        end
                    end else begin
                        // Rotation mode: rotate by angle z
                        if (z_pipe[i] > 0) begin
                            x_pipe[i+1] <= x_pipe[i] - y_shift;
                            y_pipe[i+1] <= y_pipe[i] + x_shift;
                            z_pipe[i+1] <= z_pipe[i] - atan_table[i];
                        end else begin
                            x_pipe[i+1] <= x_pipe[i] + y_shift;
                            y_pipe[i+1] <= y_pipe[i] - x_shift;
                            z_pipe[i+1] <= z_pipe[i] + atan_table[i];
                        end
                    end
                    vec_mode_pipe[i+1] <= vec_mode_pipe[i];
                end
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Output assignment
    // For vectoring: x_out = magnitude (with gain), z_out = atan2(y,x)
    // For rotation: x_out = x*cos(z)-y*sin(z), y_out = x*sin(z)+y*cos(z)
    // -------------------------------------------------------------------------
    fixed_t x_gain_comp;
    assign x_gain_comp = fixed_mul(x_pipe[NUM_ITER], K_GAIN);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_out <= FIXED_ZERO;
            y_out <= FIXED_ZERO;
            z_out <= FIXED_ZERO;
        end else if (done) begin
            if (vec_mode_pipe[NUM_ITER]) begin
                // Vectoring: compensate gain for sqrt magnitude
                x_out <= x_gain_comp; // magnitude
                y_out <= y_pipe[NUM_ITER]; // should be ~0
                z_out <= z_pipe[NUM_ITER]; // angle
            end else begin
                x_out <= x_pipe[NUM_ITER];
                y_out <= y_pipe[NUM_ITER];
                z_out <= z_pipe[NUM_ITER];
            end
        end
    end

endmodule
