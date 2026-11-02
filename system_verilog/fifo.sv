// async_fifo.sv
// 非同期クロック対応FIFO（グレイコードによる安全なCDC）
// 書き込みクロックと読み出しクロックが完全に非同期でも動作

module async_fifo #(
    parameter int DATA_WIDTH = 32,
    parameter int DEPTH      = 16,   // 2のべき乗を推奨
    parameter int ADDR_WIDTH = $clog2(DEPTH)
)(
    // 書き込み側クロックドメイン
    input  logic wclk,
    input  logic wrst_n,
    input  logic [DATA_WIDTH-1:0] wdata,
    input  logic                  wpush,
    output logic                  wfull,
    output logic                  walmost_full,

    // 読み出し側クロックドメイン
    input  logic rclk,
    input  logic rrst_n,
    output logic [DATA_WIDTH-1:0] rdata,
    input  logic                  rpop,
    output logic                  rempty,
    output logic                  ralmost_empty
);

    // メモリ本体
    logic [DATA_WIDTH-1:0] mem [DEPTH];

    // 書き込みポインタ（バイナリとグレイコード）
    logic [ADDR_WIDTH:0] wptr_bin, wptr_gray;      // 1ビット余分（フル/エンプティ判定用）
    logic [ADDR_WIDTH:0] wptr_gray_sync1, wptr_gray_sync2;  // 読み出しクロック域への同期用

    // 読み出しポインタ（バイナリとグレイコード）
    logic [ADDR_WIDTH:0] rptr_bin, rptr_gray;
    logic [ADDR_WIDTH:0] rptr_gray_sync1, rptr_gray_sync2;  // 書き込みクロック域への同期用

    //--------------------------------------------------------------------------
    // グレイコード変換関数
    //--------------------------------------------------------------------------
    function automatic logic [ADDR_WIDTH:0] bin_to_gray(logic [ADDR_WIDTH:0] bin);
        return (bin >> 1) ^ bin;
    endfunction

    function automatic logic [ADDR_WIDTH:0] gray_to_bin(logic [ADDR_WIDTH:0] gray);
        logic [ADDR_WIDTH:0] bin;
        bin[ADDR_WIDTH] = gray[ADDR_WIDTH];
        for (int i = ADDR_WIDTH-1; i >= 0; i--) begin
            bin[i] = bin[i+1] ^ gray[i];
        end
        return bin;
    endfunction

    //--------------------------------------------------------------------------
    // 書き込み側ロジック
    //--------------------------------------------------------------------------
    always_ff @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wptr_bin <= '0;
            wptr_gray <= '0;
        end else if (wpush && !wfull) begin
            mem[wptr_bin[ADDR_WIDTH-1:0]] <= wdata;
            wptr_bin  <= wptr_bin + 1'b1;
            wptr_gray <= bin_to_gray(wptr_bin + 1'b1);
        end
    end

    // 読み出しポインタを書き込みクロック域に同期
    always_ff @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            rptr_gray_sync1 <= '0;
            rptr_gray_sync2 <= '0;
        end else begin
            rptr_gray_sync1 <= rptr_gray;
            rptr_gray_sync2 <= rptr_gray_sync1;
        end
    end

    // フル判定：グレイコード比較
    // 上位2bitが反転し、残りが一致 → フル
    assign wfull = (wptr_gray == {~rptr_gray_sync2[ADDR_WIDTH:ADDR_WIDTH-1],
                                   rptr_gray_sync2[ADDR_WIDTH-2:0]});

    assign walmost_full = (wptr_bin[ADDR_WIDTH-1:0] == rptr_gray_sync2[ADDR_WIDTH-1:0]) &&
                          !wfull && !rempty;

    //--------------------------------------------------------------------------
    // 読み出し側ロジック
    //--------------------------------------------------------------------------
    always_ff @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rptr_bin <= '0;
            rptr_gray <= '0;
        end else if (rpop && !rempty) begin
            rptr_bin  <= rptr_bin + 1'b1;
            rptr_gray <= bin_to_gray(rptr_bin + 1'b1);
        end
    end

    assign rdata = mem[rptr_bin[ADDR_WIDTH-1:0]];

    // 書き込みポインタを読み出しクロック域に同期
    always_ff @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            wptr_gray_sync1 <= '0;
            wptr_gray_sync2 <= '0;
        end else begin
            wptr_gray_sync1 <= wptr_gray;
            wptr_gray_sync2 <= wptr_gray_sync1;
        end
    end

    // エンプティ判定
    assign rempty = (rptr_gray == wptr_gray_sync2);

    assign ralmost_empty = (rptr_bin[ADDR_WIDTH-1:0] + 2) >= wptr_gray_sync2[ADDR_WIDTH-1:0] &&
                           !rempty;

endmodule