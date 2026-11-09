`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: top_ik_sensor
// Description: Top-level integrating I2C Master + IK Solver
//              Reads 16-bit sensor value via I2C, maps to target (x,y),
//              then solves 2DOF inverse kinematics.
// Assumption: Sensor returns big-endian 16-bit distance in arbitrary units.
//////////////////////////////////////////////////////////////////////////////

module top_ik_sensor import fixed_math_pkg::*; (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        trigger,     // Pulse to start full sequence
    // I2C Interface
    inout  wire         scl,
    inout  wire         sda,
    // Status
    output logic        busy,
    output logic        ik_done,
    output logic        ik_valid,
    output fixed_t      theta1,
    output fixed_t      theta2,
    output logic [15:0] raw_sensor
);

    // -------------------------------------------------------------------------
    // Parameters (adjust for actual sensor and robot)
    // -------------------------------------------------------------------------
    localparam [6:0] SLAVE_ADDR = 7'h29;    // Example sensor address
    localparam [7:0] REG_ADDR   = 8'h14;    // Example measurement register
    localparam fixed_t LINK1    = int_to_fixed(100); // 100mm in Q16.16
    localparam fixed_t LINK2    = int_to_fixed(80);  // 80mm in Q16.16
    localparam int SENSOR_MAX   = 4000;     // Example sensor max range

    // -------------------------------------------------------------------------
    // I2C Master signals
    // -------------------------------------------------------------------------
    logic i2c_start;
    logic i2c_done;
    logic i2c_busy;
    logic i2c_err;
    logic [15:0] i2c_data;

    i2c_master i2c_inst (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (i2c_start),
        .slave_addr  (SLAVE_ADDR),
        .reg_addr    (REG_ADDR),
        .read_data   (i2c_data),
        .done        (i2c_done),
        .busy        (i2c_busy),
        .nack_err    (i2c_err),
        .scl         (scl),
        .sda         (sda)
    );

    // -------------------------------------------------------------------------
    // IK Solver signals
    // -------------------------------------------------------------------------
    logic ik_start;
    fixed_t target_x, target_y;

    ik_solver_2dof ik_inst (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (ik_start),
        .target_x (target_x),
        .target_y (target_y),
        .L1       (LINK1),
        .L2       (LINK2),
        .theta1   (theta1),
        .theta2   (theta2),
        .valid    (ik_valid),
        .done     (ik_done)
    );

    // -------------------------------------------------------------------------
    // Sensor value -> target position mapping
    // Example: distance mapped to X, Y fixed at mid reach
    // -------------------------------------------------------------------------
    fixed_t sensor_fixed;
    assign sensor_fixed = int_to_fixed(i2c_data); // integer portion only; scale if needed

    // -------------------------------------------------------------------------
    // Top-level control FSM
    // -------------------------------------------------------------------------
    typedef enum logic [2:0] {
        T_IDLE,
        T_I2C_START,
        T_I2C_WAIT,
        T_MAP,
        T_IK_START,
        T_IK_WAIT,
        T_FINISH
    } top_state_t;

    top_state_t top_state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            top_state <= T_IDLE;
            busy <= 0;
            i2c_start <= 0;
            ik_start <= 0;
            raw_sensor <= 0;
        end else begin
            i2c_start <= 0;
            ik_start <= 0;

            case (top_state)
                T_IDLE: begin
                    busy <= 0;
                    if (trigger) begin
                        busy <= 1;
                        top_state <= T_I2C_START;
                    end
                end

                T_I2C_START: begin
                    i2c_start <= 1;
                    top_state <= T_I2C_WAIT;
                end

                T_I2C_WAIT: begin
                    if (i2c_done) begin
                        raw_sensor <= i2c_data;
                        top_state <= T_MAP;
                    end
                end

                T_MAP: begin
                    // Map sensor to target position (example scaling)
                    // target_x = (raw / 65535) * SENSOR_MAX  -> simplified here
                    // For a real implementation, use a proper fixed-point multiplier.
                    target_x <= int_to_fixed((i2c_data * SENSOR_MAX) >> 16);
                    target_y <= int_to_fixed(60); // Fixed height example [mm]
                    top_state <= T_IK_START;
                end

                T_IK_START: begin
                    ik_start <= 1;
                    top_state <= T_IK_WAIT;
                end

                T_IK_WAIT: begin
                    if (ik_done) begin
                        busy <= 0;
                        top_state <= T_FINISH;
                    end
                end

                T_FINISH: begin
                    top_state <= T_IDLE;
                end

                default: top_state <= T_IDLE;
            endcase
        end
    end

endmodule
