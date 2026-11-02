// debounce_multi.sv
// マルチチャネル チャタリング除去回路
// 負論理ボタン（押すとLOW）を前提とした設計

module debounce_multi #(
    parameter int NUM_CHANNELS = 4,
    parameter int CLK_HZ       = 100_000_000,
    parameter int DEBOUNCE_MS  = 20
)(
    input  logic clk,
    input  logic rst_n,
    input  logic [NUM_CHANNELS-1:0] sig_in,      // 正論理化済み入力
    output logic [NUM_CHANNELS-1:0] sig_out,     // 安定した出力
    output logic [NUM_CHANNELS-1:0] posedge_detect  // 立ち上がり検出パルス
);

    localparam int COUNTER_MAX = (CLK_HZ / 1000) * DEBOUNCE_MS;
    localparam int COUNTER_W   = $clog2(COUNTER_MAX);

    logic [COUNTER_W-1:0] timer [NUM_CHANNELS];
    logic [NUM_CHANNELS-1:0] sig_sync1, sig_sync2;
    logic [NUM_CHANNELS-1:0] sig_stable;
    logic [NUM_CHANNELS-1:0] sig_prev;

    // 入力同期化（メタステーブル防止）
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sig_sync1 <= '0;
            sig_sync2 <= '0;
        end else begin
            sig_sync1 <= sig_in;
            sig_sync2 <= sig_sync1;
        end
    end

    // 各チャネル独立にチャタリング除去
    generate
        genvar i;
        for (i = 0; i < NUM_CHANNELS; i++) begin : gen_debounce
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    timer[i]      <= '0;
                    sig_stable[i] <= 1'b0;
                end else begin
                    if (sig_sync2[i] != sig_stable[i]) begin
                        timer[i] <= '0;  // 変化があればタイマーリセット
                    end else if (timer[i] < COUNTER_MAX) begin
                        timer[i] <= timer[i] + 1'b1;
                    end else begin
                        sig_stable[i] <= sig_sync2[i];  // 安定したら更新
                    end
                end
            end
        end
    endgenerate

    // 立ち上がりエッジ検出（1クロック幅パルス）
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sig_prev <= '0;
        end else begin
            sig_prev <= sig_stable;
        end
    end

    assign sig_out = sig_stable;
    assign posedge_detect = sig_stable & ~sig_prev;

endmodule
