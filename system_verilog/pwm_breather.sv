// pwm_breather.sv
// 8bit PWM 呼吸LED（ブリージング効果）
// 明るさが 0→255→0 を繰り返す

module pwm_breather #(
    parameter int CLK_HZ  = 100_000_000,
    parameter int PWM_RES = 8
)(
    input  logic clk,
    input  logic rst_n,
    input  logic [2:0] speed_sel,   // 呼吸速度選択（大きいほど速い）
    output logic pwm_out
);

    localparam int PWM_MAX    = (1 << PWM_RES) - 1;  // 255
    localparam int BASE_MS    = 10;
    localparam int BASE_TICKS = CLK_HZ / 1000 * BASE_MS;

    logic [PWM_RES-1:0] pwm_counter;
    logic [PWM_RES-1:0] brightness;
    logic [$clog2(BASE_TICKS)-1:0] fade_timer;
    logic fade_direction;  // 1:明るく 0:暗く
    logic [2:0] speed_div;

    // 速度選択（speed_selで分周比を変更）
    always_comb begin
        case (speed_sel)
            3'b000:  speed_div = 3'd0;   // 最遅
            3'b001:  speed_div = 3'd1;
            3'b010:  speed_div = 3'd2;
            3'b011:  speed_div = 3'd3;
            3'b100:  speed_div = 3'd4;
            3'b101:  speed_div = 3'd5;
            3'b110:  speed_div = 3'd6;
            3'b111:  speed_div = 3'd7;   // 最快
            default: speed_div = 3'd2;
        endcase
    end

    // PWM高速カウンタ（約390kHz @ 100MHz）
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pwm_counter <= '0;
        else
            pwm_counter <= pwm_counter + 1'b1;
    end

    // PWM比較
    assign pwm_out = (pwm_counter < brightness);

    // 明るさフェード制御
    logic [$clog2(BASE_TICKS)-1:0] fade_period;
    assign fade_period = BASE_TICKS >> speed_div;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fade_timer    <= '0;
            brightness    <= '0;
            fade_direction <= 1'b1;
        end else begin
            if (fade_timer < fade_period - 1) begin
                fade_timer <= fade_timer + 1'b1;
            end else begin
                fade_timer <= '0;
                if (fade_direction) begin
                    if (brightness < PWM_MAX)
                        brightness <= brightness + 1'b1;
                    else
                        fade_direction <= 1'b0;
                end else begin
                    if (brightness > '0)
                        brightness <= brightness - 1'b1;
                    else
                        fade_direction <= 1'b1;
                end
            end
        end
    end

endmodule
