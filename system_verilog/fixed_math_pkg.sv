`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////
// Package: fixed_math_pkg
// Description: Fixed-point math helpers (Q16.16 format)
//              32-bit signed: 16-bit integer, 16-bit fraction
//////////////////////////////////////////////////////////////////////////////

package fixed_math_pkg;

    // Q16.16 format
    typedef logic signed [31:0] fixed_t;

    // Constants in Q16.16
    localparam fixed_t FIXED_ONE  = 32'sh0001_0000; // 1.0
    localparam fixed_t FIXED_ZERO = 32'sh0000_0000; // 0.0
    localparam fixed_t PI         = 32'sh0003_243F; // 3.14159265 -> 0x0003.243F
    localparam fixed_t PI_HALF    = 32'sh0001_921F; // 1.57079633
    localparam fixed_t PI_TWO     = 32'sh0006_487F; // 6.28318531

    // Integer to fixed
    function automatic fixed_t int_to_fixed(input int x);
        return fixed_t'(x) <<< 16;
    endfunction

    // Fixed to integer (truncates fraction)
    function automatic int fixed_to_int(input fixed_t x);
        return int'(x >>> 16);
    endfunction

    // Fixed-point multiplication with rounding
    function automatic fixed_t fixed_mul(input fixed_t a, input fixed_t b);
        logic signed [63:0] temp;
        temp = (logic signed [63:0]'(a) * logic signed [63:0]'(b)) + (1 << 15); // +0.5 for rounding
        return fixed_t'(temp >>> 16);
    endfunction

    // Fixed-point division
    function automatic fixed_t fixed_div(input fixed_t a, input fixed_t b);
        if (b == 0) return FIXED_ZERO;
        logic signed [63:0] temp;
        temp = (logic signed [63:0]'(a) <<< 16) / logic signed [63:0]'(b);
        return fixed_t'(temp);
    endfunction

    // Saturate value to [-1, 1] for trig inputs
    function automatic fixed_t fixed_saturate1(input fixed_t x);
        if (x > FIXED_ONE) return FIXED_ONE;
        if (x < -FIXED_ONE) return -FIXED_ONE;
        return x;
    endfunction

endpackage
