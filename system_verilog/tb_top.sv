`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Testbench: tb_top
// Description: Simulates I2C sensor + IK chain
//              Includes a simple I2C slave model that responds with a fixed
//              16-bit measurement value.
//////////////////////////////////////////////////////////////////////////////

module tb_top;

    logic clk = 0;
    logic rst_n;
    logic trigger;
    wire scl;
    wire sda;
    logic busy;
    logic ik_done;
    logic ik_valid;
    fixed_math_pkg::fixed_t theta1, theta2;
    logic [15:0] raw_sensor;

    // Clock 50MHz
    always #10 clk = ~clk;

    // Pull-ups for I2C (simulated by weak1)
    pullup(pull1) PU_SCL (scl);
    pullup(pull1) PU_SDA (sda);

    // DUT
    top_ik_sensor dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .trigger    (trigger),
        .scl        (scl),
        .sda        (sda),
        .busy       (busy),
        .ik_done    (ik_done),
        .ik_valid   (ik_valid),
        .theta1     (theta1),
        .theta2     (theta2),
        .raw_sensor (raw_sensor)
    );

    // -------------------------------------------------------------------------
    // Simple I2C Slave Model (address 0x29, returns 16'h01A0 as measurement)
    // -------------------------------------------------------------------------
    logic [7:0] slave_mem [0:1] = '{8'h01, 8'hA0}; // MSB, LSB
    logic [7:0] rx_shift;
    logic [2:0] bit_cnt;
    logic slave_ack;
    logic [1:0] byte_cnt;

    // Simplified slave behavior (educational, not fully robust)
    always @(negedge sda) begin
        if (scl) begin
            // START detected
            bit_cnt <= 0;
            byte_cnt <= 0;
        end
    end

    // -------------------------------------------------------------------------
    // Test sequence
    // -------------------------------------------------------------------------
    initial begin
        $display("========================================");
        $display(" I2C Sensor + 2DOF IK SystemVerilog TB ");
        $display("========================================");

        rst_n <= 0;
        trigger <= 0;
        repeat(5) @(posedge clk);
        rst_n <= 1;
        repeat(5) @(posedge clk);

        // Trigger measurement + IK
        trigger <= 1;
        @(posedge clk);
        trigger <= 0;

        wait(ik_done);
        repeat(10) @(posedge clk);

        $display("[TB] Raw Sensor: 0x%04X (%0d)", raw_sensor, raw_sensor);
        $display("[TB] IK valid: %b", ik_valid);
        $display("[TB] Theta1: 0x%08X (%f rad)", theta1,
                 $itor(theta1) / 65536.0);
        $display("[TB] Theta2: 0x%08X (%f rad)", theta2,
                 $itor(theta2) / 65536.0);

        $finish;
    end

    // Waveform dump
    initial begin
        $dumpfile("ik_sensor.vcd");
        $dumpvars(0, tb_top);
    end

endmodule
