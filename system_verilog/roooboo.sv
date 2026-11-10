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
            if (led_counter == (CLK_FREQ_HZ/2 - 1)) begin // 0.5Hz点滅
                led_counter <= 25'd0;
                status_led  <= ~status_led;
            end
        end
    end

    // UART TX は現在未使用（エコーバックなどに拡張可能）
    assign uart_tx = 1'b1;

endmodule


//////////////////////////////////////////////////////////////////////////////
// UART受信モジュール (8N1)
//////////////////////////////////////////////////////////////////////////////
module uart_rx #(
    parameter CLK_FREQ_HZ = 50_000_000,
    parameter BAUDRATE    = 115200
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        rx_pin,
    output logic [7:0]  rx_data,
    output logic        rx_valid
);

    localparam BIT_PERIOD = CLK_FREQ_HZ / BAUDRATE;

    typedef enum logic [2:0] {
        IDLE,
        START_BIT,
        DATA_BITS,
        STOP_BIT,
        DONE
    } state_t;

    state_t state;
    logic [15:0]  baud_counter;
    logic [2:0]   bit_index;
    logic [7:0]   rx_shift_reg;
    logic         rx_sync, rx_sync2;

    // メタステーブル対策の2段同期
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync  <= 1'b1;
            rx_sync2 <= 1'b1;
        end else begin
            rx_sync  <= rx_pin;
            rx_sync2 <= rx_sync;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= IDLE;
            baud_counter  <= 16'd0;
            bit_index     <= 3'd0;
            rx_shift_reg  <= 8'd0;
            rx_data       <= 8'd0;
            rx_valid      <= 1'b0;
        end else begin
            rx_valid <= 1'b0; // デフォルトで無効

            case (state)
                IDLE: begin
                    baud_counter <= 16'd0;
                    bit_index    <= 3'd0;
                    if (rx_sync2 == 1'b0) begin // スタートビット検出
                        state <= START_BIT;
                    end
                end

                START_BIT: begin
                    if (baud_counter == (BIT_PERIOD / 2)) begin
                        if (rx_sync2 == 1'b0) begin // 確認
                            baud_counter <= 16'd0;
                            state        <= DATA_BITS;
                        end else begin
                            state <= IDLE;
                        end
                    end else begin
                        baud_counter <= baud_counter + 16'd1;
                    end
                end

                DATA_BITS: begin
                    if (baud_counter == (BIT_PERIOD - 1)) begin
                        baud_counter <= 16'd0;
                        rx_shift_reg <= {rx_sync2, rx_shift_reg[7:1]};
                        if (bit_index == 3'd7) begin
                            state <= STOP_BIT;
                        end else begin
                            bit_index <= bit_index + 3'd1;
                        end
                    end else begin
                        baud_counter <= baud_counter + 16'd1;
                    end
                end

                STOP_BIT: begin
                    if (baud_counter == (BIT_PERIOD - 1)) begin
                        rx_data  <= rx_shift_reg;
                        rx_valid <= 1'b1;
                        state    <= IDLE;
                    end else begin
                        baud_counter <= baud_counter + 16'd1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule


//////////////////////////////////////////////////////////////////////////////
// PWM生成モジュール (8bit分解能, 0-255)
//////////////////////////////////////////////////////////////////////////////
module pwm_generator #(
    parameter CLK_FREQ_HZ = 50_000_000,
    parameter PWM_FREQ_HZ = 20_000
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic [7:0]  duty,
    output logic        pwm_out
);

    localparam PWM_PERIOD = CLK_FREQ_HZ / PWM_FREQ_HZ / 256;
    
    logic [15:0] prescaler;
    logic [7:0]  counter;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prescaler <= 16'd0;
            counter   <= 8'd0;
            pwm_out   <= 1'b0;
        end else begin
            if (prescaler == (PWM_PERIOD - 1)) begin
                prescaler <= 16'd0;
                counter   <= counter + 8'd1;
            end else begin
                prescaler <= prescaler + 16'd1;
            end
            
            pwm_out <= (counter < duty);
        end
    end

endmodule