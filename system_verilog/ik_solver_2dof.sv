`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: ik_solver_2dof
// Description: Complete 2-DOF planar inverse kinematics (Q16.16 fixed-point)
// Posture: elbow-down (sin(theta2) < 0)
// Uses: cordic (atan2, magnitude), fixed_sqrt, fixed_mul/div
//////////////////////////////////////////////////////////////////////////////

module ik_solver_2dof import fixed_math_pkg::*; (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  fixed_t      target_x,   // Q16.16 [mm]
    input  fixed_t      target_y,   // Q16.16 [mm]
    input  fixed_t      L1,         // Q16.16 [mm]
    input  fixed_t      L2,         // Q16.16 [mm]
    output fixed_t      theta1,     // Q16.16 [rad]
    output fixed_t      theta2,     // Q16.16 [rad]
    output logic        valid,
    output logic        done
);

    // -------------------------------------------------------------------------
    // Sub-modules control signals
    // -------------------------------------------------------------------------
    logic c1_start, c1_done;
    fixed_t c1_xout, c1_zout;

    logic sqrt_start, sqrt_done;
    fixed_t sqrt_in, sqrt_out;

    logic c2_start, c2_done;
    fixed_t c2_zout;

    logic c3_start, c3_done;
    fixed_t c3_zout;

    // -------------------------------------------------------------------------
    // Sub-module instances
    // -------------------------------------------------------------------------
    cordic cordic1 (
        .clk   (clk),
        .rst_n (rst_n),
        .start (c1_start),
        .mode  (1'b0),              // vectoring: atan2 + magnitude
        .x_in  (target_x),
        .y_in  (target_y),
        .z_in  (FIXED_ZERO),
        .x_out (c1_xout),
        .y_out (),
        .z_out (c1_zout),
        .done  (c1_done)
    );

    fixed_sqrt sqrt1 (
        .clk   (clk),
        .rst_n (rst_n),
        .start (sqrt_start),
        .in    (sqrt_in),
        .out   (sqrt_out),
        .done  (sqrt_done)
    );

    // CORDIC2: atan2(sin2, cos2) -> theta2
    cordic cordic2 (
        .clk   (clk),
        .rst_n (rst_n),
        .start (c2_start),
        .mode  (1'b0),
        .x_in  (cos2_reg),
        .y_in  (sin2_reg),
        .z_in  (FIXED_ZERO),
        .x_out (),
        .y_out (),
        .z_out (c2_zout),
        .done  (c2_done)
    );

    // CORDIC3: atan2(L2*sin2, L1 + L2*cos2) -> beta
    cordic cordic3 (
        .clk   (clk),
        .rst_n (rst_n),
        .start (c3_start),
        .mode  (1'b0),
        .x_in  (beta_x_reg),
        .y_in  (beta_y_reg),
        .z_in  (FIXED_ZERO),
        .x_out (),
        .y_out (),
        .z_out (c3_zout),
        .done  (c3_done)
    );

    // -------------------------------------------------------------------------
    // IK computation registers
    // -------------------------------------------------------------------------
    fixed_t r_mag, alpha;
    fixed_t cos2, sin2;
    fixed_t cos2_reg, sin2_reg;
    fixed_t beta_x_reg, beta_y_reg;
    fixed_t L1_plus_L2cos2;
    logic reach_ok;

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        S_IDLE,
        S_C1,       // atan2(y,x) & r
        S_C1_WAIT,
        S_CHECK,
        S_COS2,     // cosine law
        S_SIN2_SQ,  // 1 - cos^2
        S_SQRT,     // sqrt for sin magnitude
        S_SQ_WAIT,
        S_NEG_SIN,  // elbow-down: sin = -sqrt(...)
        S_C2,       // atan2(sin2, cos2)
        S_C2_WAIT,
        S_BETA_PREP,// prepare beta args
        S_C3,       // atan2(L2*sin2, L1+L2*cos2)
        S_C3_WAIT,
        S_THETA1,   // theta1 = alpha - beta
        S_DONE
    } state_t;

    state_t state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 0;
            valid <= 0;
            theta1 <= FIXED_ZERO;
            theta2 <= FIXED_ZERO;
            c1_start <= 0;
            sqrt_start <= 0;
            c2_start <= 0;
            c3_start <= 0;
        end else begin
            c1_start <= 0;
            sqrt_start <= 0;
            c2_start <= 0;
            c3_start <= 0;
            done <= 0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        valid <= 0;
                        state <= S_C1;
                    end
                end

                S_C1: begin
                    c1_start <= 1;
                    state <= S_C1_WAIT;
                end

                S_C1_WAIT: begin
                    if (c1_done) begin
                        r_mag <= c1_xout;
                        alpha <= c1_zout;
                        state <= S_CHECK;
                    end
                end

                S_CHECK: begin
                    fixed_t r_max = L1 + L2;
                    fixed_t r_min = (L1 > L2) ? (L1 - L2) : (L2 - L1);
                    if (r_mag > r_max || r_mag < r_min || L1 <= 0 || L2 <= 0) begin
                        valid <= 0;
                        state <= S_DONE;
                    end else begin
                        valid <= 1;
                        state <= S_COS2;
                    end
                end

                S_COS2: begin
                    // cos2 = (r^2 - L1^2 - L2^2) / (2*L1*L2)
                    fixed_t r_sq = fixed_mul(r_mag, r_mag);
                    fixed_t L1_sq = fixed_mul(L1, L1);
                    fixed_t L2_sq = fixed_mul(L2, L2);
                    fixed_t num = r_sq - L1_sq - L2_sq;
                    fixed_t den = fixed_mul(L1, L2) <<< 1;
                    cos2 <= fixed_saturate1(fixed_div(num, den));
                    state <= S_SIN2_SQ;
                end

                S_SIN2_SQ: begin
                    fixed_t c2sq = fixed_mul(cos2, cos2);
                    fixed_t one_minus = FIXED_ONE - c2sq;
                    if (one_minus < 0) one_minus = 0;
                    sqrt_in <= one_minus;
                    sqrt_start <= 1;
                    state <= S_SQ_WAIT;
                end

                S_SQ_WAIT: begin
                    if (sqrt_done) begin
                        // elbow-down => negative sin
                        sin2 <= -sqrt_out;
                        cos2_reg <= cos2;
                        state <= S_C2;
                    end
                end

                S_C2: begin
                    c2_start <= 1;
                    state <= S_C2_WAIT;
                end

                S_C2_WAIT: begin
                    if (c2_done) begin
                        theta2 <= c2_zout;
                        // Prepare beta = atan2(L2*sin2, L1 + L2*cos2)
                        beta_y_reg <= fixed_mul(L2, sin2);
                        beta_x_reg <= L1 + fixed_mul(L2, cos2_reg);
                        state <= S_C3;
                    end
                end

                S_C3: begin
                    c3_start <= 1;
                    state <= S_C3_WAIT;
                end

                S_C3_WAIT: begin
                    if (c3_done) begin
                        // theta1 = alpha - beta
                        theta1 <= alpha - c3_zout;
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    done <= 1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
