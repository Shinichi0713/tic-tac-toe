// riscv_alu.sv
// RISC-V RV32I準拠のパイプライン化ALU
// 1サイクルレイテンシのパイプラインALU

module riscv_alu #(
    parameter int XLEN = 32
)(
    input  logic clk,
    input  logic rst_n,
    input  logic valid_in,            // 入力有効
    output logic ready_out,           // 出力有効（1サイクル後）

    // 制御信号（デコード済み）
    input  logic [3:0]  alu_op,       // 演算種別
    input  logic [XLEN-1:0] rs1_data,
    input  logic [XLEN-1:0] rs2_data,
    input  logic        use_imm,      // 即値使用フラグ
    input  logic [XLEN-1:0] imm_data,

    // 結果出力
    output logic [XLEN-1:0] alu_result,
    output logic            zero_flag,
    output logic            negative_flag,
    output logic            overflow_flag
);

    // ALU操作コード定義
    localparam logic [3:0] ALU_ADD  = 4'b0000;
    localparam logic [3:0] ALU_SUB  = 4'b1000;
    localparam logic [3:0] ALU_SLL  = 4'b0001;  // Shift Left Logical
    localparam logic [3:0] ALU_SLT  = 4'b0010;  // Set Less Than (signed)
    localparam logic [3:0] ALU_SLTU = 4'b0011;  // Set Less Than Unsigned
    localparam logic [3:0] ALU_XOR  = 4'b0100;
    localparam logic [3:0] ALU_SRL  = 4'b0101;  // Shift Right Logical
    localparam logic [3:0] ALU_SRA  = 4'b1101;  // Shift Right Arithmetic
    localparam logic [3:0] ALU_OR   = 4'b0110;
    localparam logic [3:0] ALU_AND  = 4'b0111;

    // パイプラインレジスタ
    logic [XLEN-1:0] operand_a, operand_b;
    logic [3:0]      alu_op_reg;
    logic            valid_reg;

    // 入力ラッチ（パイプライン第1段）
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_reg <= 1'b0;
        end else begin
            valid_reg <= valid_in;
            if (valid_in) begin
                operand_a <= rs1_data;
                operand_b <= use_imm ? imm_data : rs2_data;
                alu_op_reg <= alu_op;
            end
        end
    end

    assign ready_out = valid_reg;

    //--------------------------------------------------------------------------
    // 組み合わせ論理：ALU演算本体
    //--------------------------------------------------------------------------
    logic [XLEN-1:0] result_comb;
    logic            is_add_sub;
    logic [XLEN-1:0] adder_operand_b;
    logic            adder_cin;
    logic [XLEN-1:0] adder_result;
    logic            adder_cout;

    // ADD/SUBは同じ加算器で実装（SUBはBのビット反転+1）
    assign is_add_sub = (alu_op_reg == ALU_ADD) || (alu_op_reg == ALU_SUB);
    assign adder_operand_b = (alu_op_reg == ALU_SUB) ? ~operand_b : operand_b;
    assign adder_cin       = (alu_op_reg == ALU_SUB) ? 1'b1 : 1'b0;

    // 加算器
    assign {adder_cout, adder_result} = operand_a + adder_operand_b + adder_cin;

    always_comb begin
        case (alu_op_reg)
            ALU_ADD:  result_comb = adder_result;
            ALU_SUB:  result_comb = adder_result;
            ALU_SLL:  result_comb = operand_a << operand_b[4:0];  // RV32Iでは最大31シフト
            ALU_SLT:  result_comb = ($signed(operand_a) < $signed(operand_b)) ? 32'd1 : 32'd0;
            ALU_SLTU: result_comb = (operand_a < operand_b) ? 32'd1 : 32'd0;
            ALU_XOR:  result_comb = operand_a ^ operand_b;
            ALU_SRL:  result_comb = operand_a >> operand_b[4:0];
            ALU_SRA:  result_comb = $signed(operand_a) >>> operand_b[4:0];
            ALU_OR:   result_comb = operand_a | operand_b;
            ALU_AND:  result_comb = operand_a & operand_b;
            default:  result_comb = '0;
        endcase
    end

    // 出力レジスタ（パイプライン第2段）
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            alu_result <= '0;
        end else if (valid_reg) begin
            alu_result <= result_comb;
        end
    end

    // フラグ出力
    assign zero_flag     = (alu_result == '0);
    assign negative_flag = alu_result[XLEN-1];

    // オーバーフロー判定（符号付き加減算時のみ）
    assign overflow_flag = is_add_sub &&
                           (operand_a[XLEN-1] == adder_operand_b[XLEN-1]) &&
                           (alu_result[XLEN-1] != operand_a[XLEN-1]);

endmodule