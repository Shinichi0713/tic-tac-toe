//////////////////////////////////////////////////////////////////////////////
// ロボット制御トップモジュール (差動駆動2輪ロボット)
//////////////////////////////////////////////////////////////////////////////
module robot_controller #(
    parameter CLK_FREQ_HZ = 50_000_000,  // システムクロック周波数
    parameter PWM_FREQ_HZ = 20_000,      // PWM周波数
    parameter UART_BAUDRATE = 115200     // UARTボーレート
)(
    input  logic        clk,
    input  logic        rst_n,

    // UARTインターフェース
    input  logic        uart_rx,
    output logic        uart_tx,

    // 左モーター制御
    output logic        left_pwm,
    output logic        left_dir,   // 0:正転, 1:逆転
    output logic        left_brake,

    // 右モーター制御
    output logic        right_pwm,
    output logic        right_dir,
    output logic        right_brake,

    // ステータスLED
    output logic        status_led
);

    //=========================================================================
    // 内部信号
    //=========================================================================
    logic [7:0]  uart_rx_data;
    logic        uart_rx_valid;
    logic [7:0]  speed_left, speed_right;
    logic        dir_left, dir_right;
    logic        brake;

    //=========================================================================
    // UART受信モジュール
    //=========================================================================
    uart_rx #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .BAUDRATE(UART_BAUDRATE)
    ) u_uart_rx (
        .clk        (clk),
        .rst_n      (rst_n),
        .rx_pin     (uart_rx),
        .rx_data    (uart_rx_data),
        .rx_valid   (uart_rx_valid)
    );

    //=========================================================================
    // コマンドデコーダ
    // コマンドフォーマット: [7:6]コマンド, [5:0]データ
    //  2'b00: 左モータースピード (0-63)
    //  2'b01: 右モータースピード (0-63)
    //  2'b10: 方向制御 {左方向, 右方向, ブレーキ, 予約3bit}
    //  2'b11: 予約
    //=========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            speed_left  <= 8'd0;
            speed_right <= 8'd0;
            dir_left    <= 1'b0;
            dir_right   <= 1'b0;
            brake       <= 1'b0;
        end else if (uart_rx_valid) begin
            case (uart_rx_data[7:6])
                2'b00: speed_left  <= {uart_rx_data[5:0], 2'b00}; // 6bit -> 8bit
                2'b01: speed_right <= {uart_rx_data[5:0], 2'b00};
                2'b10: begin
                    dir_left  <= uart_rx_data[5];
                    dir_right <= uart_rx_data[4];
                    brake     <= uart_rx_data[3];
                end
                default: ;
            endcase
        end
    end

    //=========================================================================
    // PWM生成モジュール（左右独立）
    //=========================================================================
    pwm_generator #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .PWM_FREQ_HZ(PWM_FREQ_HZ)
    ) u_pwm_left (
        .clk        (clk),
        .rst_n      (rst_n),
        .duty       (speed_left),
        .pwm_out    (left_pwm)
    );

    pwm_generator #(
        .CLK_FREQ_HZ(CLK_FREQ_HZ),
        .PWM_FREQ_HZ(PWM_FREQ_HZ)
    ) u_pwm_right (
        .clk        (clk),
        .rst_n      (rst_n),
        .duty       (speed_right),
        .pwm_out    (right_pwm)
    );

    // 方向・ブレーキ出力
    assign left_dir    = dir_left;
    assign right_dir   = dir_right;
    assign left_brake  = brake;
    assign right_brake = brake;

    // ステータスLED（動作中に点滅）
    logic [24:0] led_counter;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            led_counter <= 25'd0;
            status_led  <= 1'b0;
        end else begin
            led_counter <= led_counter + 25'd1;
            if (led_counter == (CLK_FREQ_HZ/2 - 1)) b