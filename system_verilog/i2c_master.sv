`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Module: i2c_master
// Description: Simplified I2C Master for 2-byte register read
//              7-bit slave address, 8-bit register address,
//              Repeated START, then MSB + LSB read.
// Clock: 50MHz input -> 100kHz I2C (divider = 500)
// Usage: Pulse 'start' -> wait 'done' -> read 'read_data'
//////////////////////////////////////////////////////////////////////////////

module i2c_master (
    input  logic clk,           // System clock (e.g., 50MHz)
    input  logic rst_n,         // Active-low reset
    // Control Interface
    input  logic start,         // Pulse high to initiate
    input  logic [6:0] slave_addr,
    input  logic [7:0] reg_addr,
    output logic [15:0] read_data,
    output logic done,          // High for one I2C cycle when finished
    output logic busy,
    output logic nack_err,      // High if slave NACKed
    // I2C Bus (external pull-ups required on board)
    inout  wire  scl,
    inout  wire  sda
);

    // -------------------------------------------------------------------------
    // Clock divider: 50 MHz -> 100 kHz (I2C bit period = 10 us)
    // -------------------------------------------------------------------------
    localparam DIV_CNT = 9'd500;        // 50MHz / 100kHz = 500
    localparam DIV_HALF = 9'd250;       // Quarter/half markers

    logic [8:0] div_cnt;
    logic tick_q;                       // Quarter phase tick
    logic tick_h;                       // Half phase tick
    logic tick_3q;                      // Three-quarter phase tick

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 0;
        end else begin
            if (div_cnt == DIV_CNT - 1)
                div_cnt <= 0;
            else
                div_cnt <= div_cnt + 1;
        end
    end

    assign tick_q  = (div_cnt == DIV_HALF/2 - 1);      // 25%
    assign tick_h  = (div_cnt == DIV_HALF - 1);        // 50%
    assign tick_3q = (div_cnt == DIV_HALF + DIV_HALF/2 - 1); // 75%

    // -------------------------------------------------------------------------
    // I2C line drivers: 0 = drive low, 1 = high-Z (rely on external pull-up)
    // -------------------------------------------------------------------------
    logic scl_drive_low;
    logic sda_drive_low;

    assign scl = scl_drive_low ? 1'b0 : 1'bz;
    assign sda = sda_drive_low ? 1'b0 : 1'bz;

    // -------------------------------------------------------------------------
    // State Machine
    // -------------------------------------------------------------------------
    typedef enum logic [5:0] {
        S_IDLE,
        S_START_1,          // SDA low while SCL high
        S_START_2,
        S_ADDR_WR,          // Shift out 8 bits: {slave_addr, 1'b0}
        S_ADDR_WR_ACK,      // Read ACK bit
        S_REG_ADDR,
        S_REG_ADDR_ACK,
        S_REP_START_1,
        S_REP_START_2,
        S_ADDR_RD,
        S_ADDR_RD_ACK,
        S_RD_H,             // Read high byte
        S_RD_H_ACK,         // Master sends ACK to slave
        S_RD_L,             // Read low byte
        S_RD_L_NACK,        // Master sends NACK
        S_STOP_1,
        S_STOP_2,
        S_DONE
    } state_t;

    state_t state;

    logic [3:0] bit_cnt;
    logic [7:0] tx_shift;
    logic [7:0] rx_shift;
    logic [15:0] rx_buffer;

    // -------------------------------------------------------------------------
    // Sequential Logic
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            scl_drive_low <= 0;
            sda_drive_low <= 0;
            bit_cnt <= 0;
            tx_shift <= 0;
            rx_shift <= 0;
            rx_buffer <= 0;
            read_data <= 0;
            done <= 0;
            busy <= 0;
            nack_err <= 0;
        end else begin
            done <= 0; // default pulse off

            case (state)
                // -----------------------------------------------------------------
                S_IDLE: begin
                    scl_drive_low <= 0; // release (high by pull-up)
                    sda_drive_low <= 0; // release
                    busy <= 0;
                    if (start) begin
                        busy <= 1;
                        nack_err <= 0;
                        tx_shift <= {slave_addr, 1'b0};
                        state <= S_START_1;
                    end
                end

                // -----------------------------------------------------------------
                // START condition: SDA goes low while SCL is high
                // -----------------------------------------------------------------
                S_START_1: begin
                    if (tick_h) begin
                        sda_drive_low <= 1; // pull SDA low
                        state <= S_START_2;
                    end
                end
                S_START_2: begin
                    if (tick_3q) begin
                        scl_drive_low <= 1; // pull SCL low
                        bit_cnt <= 7;
                        state <= S_ADDR_WR;
                    end
                end

                // -----------------------------------------------------------------
                // Write Address + W (8 bits)
                // Clock pattern: SCL low -> place SDA -> SCL high -> SCL low
                // -----------------------------------------------------------------
                S_ADDR_WR: begin
                    if (tick_q) begin
                        // SCL is already low here. Place data bit.
                        sda_drive_low <= ~tx_shift[7]; // MSB first
                    end else if (tick_h) begin
                        scl_drive_low <= 0; // release SCL high
                    end else if (tick_3q) begin
                        scl_drive_low <= 1; // pull SCL low
                        if (bit_cnt == 0) begin
                            state <= S_ADDR_WR_ACK;
                            sda_drive_low <= 0; // release SDA for slave ACK
                        end else begin
                            tx_shift <= {tx_shift[6:0], 1'b0};
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end

                // -----------------------------------------------------------------
                // Read ACK after address write
                // -----------------------------------------------------------------
                S_ADDR_WR_ACK: begin
                    if (tick_h) begin
                        // Sample SDA. 0 = ACK, 1 = NACK
                        if (sda != 1'b0) begin
                            nack_err <= 1;
                            state <= S_STOP_1; // abort
                        end else begin
                            tx_shift <= reg_addr;
                            bit_cnt <= 7;
                            state <= S_REG_ADDR;
                        end
                    end else if (tick_3q) begin
                        scl_drive_low <= 1; // pull SCL low
                    end
                end

                // -----------------------------------------------------------------
                // Write Register Address (8 bits)
                // -----------------------------------------------------------------
                S_REG_ADDR: begin
                    if (tick_q) begin
                        sda_drive_low <= ~tx_shift[7];
                    end else if (tick_h) begin
                        scl_drive_low <= 0;
                    end else if (tick_3q) begin
                        scl_drive_low <= 1;
                        if (bit_cnt == 0) begin
                            state <= S_REG_ADDR_ACK;
                            sda_drive_low <= 0;
                        end else begin
                            tx_shift <= {tx_shift[6:0], 1'b0};
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end

                S_REG_ADDR_ACK: begin
                    if (tick_h) begin
                        if (sda != 1'b0) begin
                            nack_err <= 1;
                            state <= S_STOP_1;
                        end else begin
                            state <= S_REP_START_1;
                        end
                    end else if (tick_3q) begin
                        scl_drive_low <= 1;
                    end
                end

                // -----------------------------------------------------------------
                // Repeated START
                // -----------------------------------------------------------------
                S_REP_START_1: begin
                    if (tick_h) begin
                        sda_drive_low <= 0; // SDA was already released, now drive low
                        scl_drive_low <= 0; // SCL stays high (released)
                        state <= S_REP_START_2;
                    end
                end
                S_REP_START_2: begin
                    if (tick_3q) begin
                        scl_drive_low <= 1; // pull SCL low
                        tx_shift <= {slave_addr, 1'b1}; // R/W = 1 (read)
                        bit_cnt <= 7;
                        state <= S_ADDR_RD;
                    end
                end

                // -----------------------------------------------------------------
                // Write Address + R
                // -----------------------------------------------------------------
                S_ADDR_RD: begin
                    if (tick_q) begin
                        sda_drive_low <= ~tx_shift[7];
                    end else if (tick_h) begin
                        scl_drive_low <= 0;
                    end else if (tick_3q) begin
                        scl_drive_low <= 1;
                        if (bit_cnt == 0) begin
                            state <= S_ADDR_RD_ACK;
                            sda_drive_low <= 0; // release for slave ACK
                        end else begin
                            tx_shift <= {tx_shift[6:0], 1'b0};
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end

                S_ADDR_RD_ACK: begin
                    if (tick_h) begin
                        if (sda != 1'b0) begin
                            nack_err <= 1;
                            state <= S_STOP_1;
                        end else begin
                            bit_cnt <= 7;
                            state <= S_RD_H;
                        end
                    end else if (tick_3q) begin
                        scl_drive_low <= 1;
                    end
                end

                // -----------------------------------------------------------------
                // Read High Byte (MSB)
                // -----------------------------------------------------------------
                S_RD_H: begin
                    if (tick_q) begin
                        sda_drive_low <= 0; // release, slave drives
                    end else if (tick_h) begin
                        scl_drive_low <= 0; // SCL high, sample soon
                        rx_shift <= {rx_shift[6:0], sda}; // shift in (LSB first capture, will reorder)
                    end else if (tick_3q) begin
                        scl_drive_low <= 1; // SCL low
                        if (bit_cnt == 0) begin
                            rx_buffer[15:8] <= rx_shift;
                            state <= S_RD_H_ACK;
                            sda_drive_low <= 0; // prepare ACK (drive low)
                        end else begin
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end

                S_RD_H_ACK: begin
                    // Master sends ACK (SDA low) so slave continues
                    if (tick_q) begin
                        sda_drive_low <= 1; // ACK = low
                    end else if (tick_h) begin
                        scl_drive_low <= 0; // SCL high
                    end else if (tick_3q) begin
                        scl_drive_low <= 1; // SCL low
                        bit_cnt <= 7;
                        state <= S_RD_L;
                    end
                end

                // -----------------------------------------------------------------
                // Read Low Byte (LSB)
                // -----------------------------------------------------------------
                S_RD_L: begin
                    if (tick_q) begin
                        sda_drive_low <= 0;
                    end else if (tick_h) begin
                        scl_drive_low <= 0;
                        rx_shift <= {rx_shift[6:0], sda};
                    end else if (tick_3q) begin
                        scl_drive_low <= 1;
                        if (bit_cnt == 0) begin
                            rx_buffer[7:0] <= rx_shift;
                            state <= S_RD_L_NACK;
                            sda_drive_low <= 1; // prepare NACK (drive high -> release)
                        end else begin
                            bit_cnt <= bit_cnt - 1;
                        end
                    end
                end

                S_RD_L_NACK: begin
                    // Master sends NACK (SDA high / released)
                    if (tick_q) begin
                        sda_drive_low <= 0; // release for NACK
                    end else if (tick_h) begin
                        scl_drive_low <= 0;
                    end else if (tick_3q) begin
                        scl_drive_low <= 1;
                        state <= S_STOP_1;
                    end
                end

                // -----------------------------------------------------------------
                // STOP condition: SDA goes high while SCL is high
                // -----------------------------------------------------------------
                S_STOP_1: begin
                    if (tick_q) begin
                        sda_drive_low <= 1; // ensure SDA low
                    end else if (tick_h) begin
                        scl_drive_low <= 0; // release SCL high
                    end else if (tick_3q) begin
                        sda_drive_low <= 0; // release SDA high (while SCL high)
                        state <= S_STOP_2;
                    end
                end
                S_STOP_2: begin
                    if (tick_h) begin
                        state <= S_DONE;
                    end
                end

                // -----------------------------------------------------------------
                S_DONE: begin
                    read_data <= rx_buffer;
                    done <= 1;
                    busy <= 0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
