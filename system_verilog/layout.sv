// axi4lite_reg_slave.sv
// AXI4-Lite準拠のレジスタスレーブ（4個の読み書きレジスタ）
// 実際のSoC設計で使用される構造を再現

module axi4lite_reg_slave #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32,
    parameter int NUM_REGS   = 4    // 4つのレジスタ（0x00, 0x04, 0x08, 0x0C）
)(
    input  logic aclk,
    input  logic areset_n,          // AXIは負論理リセットが標準

    // AXI4-Lite Write Address Channel
    input  logic [ADDR_WIDTH-1:0] awaddr,
    input  logic                  awvalid,
    output logic                  awready,

    // AXI4-Lite Write Data Channel
    input  logic [DATA_WIDTH-1:0] wdata,
    input  logic [DATA_WIDTH/8-1:0] wstrb,  // バイト単位書き込みマスク
    input  logic                  wvalid,
    output logic                  wready,

    // AXI4-Lite Write Response Channel
    output logic [1:0]            bresp,    // OKAY/SLVERR
    output logic                    bvalid,
    input  logic                    bready,

    // AXI4-Lite Read Address Channel
    input  logic [ADDR_WIDTH-1:0] araddr,
    input  logic                    arvalid,
    output logic                    arready,

    // AXI4-Lite Read Data Channel
    output logic [DATA_WIDTH-1:0] rdata,
    output logic [1:0]              rresp,
    output logic                    rvalid,
    input  logic                    rready,

    // 外部へ出力するレジスタ値
    output logic [DATA_WIDTH-1:0] reg_out [NUM_REGS]
);

    // AXI応答コード
    localparam logic [1:0] RESP_OKAY   = 2'b00;
    localparam logic [1:0] RESP_SLVERR  = 2'b10;

    // レジスタ本体
    logic [DATA_WIDTH-1:0] regs [NUM_REGS];

    // アドレスの有効範囲チェック
    function automatic logic addr_valid(logic [ADDR_WIDTH-1:0] addr);
        return (addr < (NUM_REGS * 4));  // 各レジスタ4バイト
    endfunction

    // レジスタインデックス取得
    function automatic logic [$clog2(NUM_REGS)-1:0] addr_to_idx(logic [ADDR_WIDTH-1:0] addr);
        return addr[2 +: $clog2(NUM_REGS)];  // 下位2bitはバイトオフセット
    endfunction

    //--------------------------------------------------------------------------
    // 書き込みチャネル状態機械
    //--------------------------------------------------------------------------
    typedef enum logic [1:0] {
        WR_IDLE,
        WR_ADDR,
        WR_DATA,
        WR_RESP
    } wr_state_t;

    wr_state_t wr_state, wr_state_next;
    logic [ADDR_WIDTH-1:0] awaddr_latch;
    logic [DATA_WIDTH-1:0] wdata_latch;
    logic [DATA_WIDTH/8-1:0] wstrb_latch;

    always_ff @(posedge aclk or negedge areset_n) begin
        if (!areset_n) wr_state <= WR_IDLE;
        else           wr_state <= wr_state_next;
    end

    always_comb begin
        wr_state_next = wr_state;
        case (wr_state)
            WR_IDLE: if (awvalid) wr_state_next = WR_ADDR;
            WR_ADDR: if (wvalid)  wr_state_next = WR_DATA;
            WR_DATA: if (bready)  wr_state_next = WR_RESP;
            WR_RESP:              wr_state_next = WR_IDLE;
            default: ;
        endcase
    end

    assign awready = (wr_state == WR_IDLE);
    assign wready  = (wr_state == WR_ADDR);

    always_ff @(posedge aclk) begin
        if (awvalid && awready) awaddr_latch <= awaddr;
        if (wvalid  && wready)  begin
            wdata_latch <= wdata;
            wstrb_latch <= wstrb;
        end
    end

    // レジスタ書き込み（バイトストローブ対応）
    always_ff @(posedge aclk) begin
        if (wr_state == WR_DATA) begin
            if (addr_valid(awaddr_latch)) begin
                automatic logic [$clog2(NUM_REGS)-1:0] idx = addr_to_idx(awaddr_latch);
                for (int i = 0; i < DATA_WIDTH/8; i++) begin
                    if (wstrb_latch[i]) begin
                        regs[idx][i*8 +: 8] <= wdata_latch[i*8 +: 8];
                    end
                end
            end
        end
    end

    assign bvalid = (wr_state == WR_DATA);
    assign bresp  = addr_valid(awaddr_latch) ? RESP_OKAY : RESP_SLVERR;

    //--------------------------------------------------------------------------
    // 読み出しチャネル状態機械
    //--------------------------------------------------------------------------
    typedef enum logic [1:0] {
        RD_IDLE,
        RD_ADDR,
        RD_DATA
    } rd_state_t;

    rd_state_t rd_state, rd_state_next;
    logic [ADDR_WIDTH-1:0] araddr_latch;

    always_ff @(posedge aclk or negedge areset_n) begin
        if (!areset_n) rd_state <= RD_IDLE;
        else           rd_state <= rd_state_next;
    end

    always_comb begin
        rd_state_next = rd_state;
        case (rd_state)
            RD_IDLE: if (arvalid) rd_state_next = RD_ADDR;
            RD_ADDR:             rd_state_next = RD_DATA;
            RD_DATA: if (rready) rd_state_next = RD_IDLE;
            default: ;
        endcase
    end

    assign arready = (rd_state == RD_IDLE);
    assign rvalid  = (rd_state == RD_DATA);

    always_ff @(posedge aclk) begin
        if (arvalid && arready) araddr_latch <= araddr;
    end

    always_comb begin
        if (addr_valid(araddr_latch)) begin
            rdata = regs[addr_to_idx(araddr_latch)];
            rresp = RESP_OKAY;
        end else begin
            rdata = '0;
            rresp = RESP_SLVERR;
        end
    end

    // 外部出力
    generate
        genvar g;
        for (g = 0; g < NUM_REGS; g++) begin : gen_reg_out
            assign reg_out[g] = regs[g];
        end
    endgenerate

endmodule

