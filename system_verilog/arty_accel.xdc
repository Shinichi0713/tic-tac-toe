## arty_accel.xdc
## Arty A7 + Pmod ACL (ADXL345) 加速度センサ 用制約ファイル

##===========================================================================
## コンフィグ設定
##===========================================================================
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

##===========================================================================
## クロック信号 (100MHz)
##===========================================================================
set_property -dict { PACKAGE_PIN E3 IOSTANDARD LVCMOS33 } [get_ports { CLK100MHZ }];
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports {CLK100MHZ}];

##===========================================================================
## スライドスイッチ (4個)
##===========================================================================
set_property -dict { PACKAGE_PIN A8  IOSTANDARD LVCMOS33 } [get_ports { SW[0] }];
set_property -dict { PACKAGE_PIN C11 IOSTANDARD LVCMOS33 } [get_ports { SW[1] }];
set_property -dict { PACKAGE_PIN C10 IOSTANDARD LVCMOS33 } [get_ports { SW[2] }];
set_property -dict { PACKAGE_PIN A10 IOSTANDARD LVCMOS33 } [get_ports { SW[3] }];

##===========================================================================
## プッシュボタン (4個、負論理)
##===========================================================================
set_property -dict { PACKAGE_PIN D9 IOSTANDARD LVCMOS33 } [get_ports { BTN[0] }];
set_property -dict { PACKAGE_PIN C9 IOSTANDARD LVCMOS33 } [get_ports { BTN[1] }];
set_property -dict { PACKAGE_PIN B9 IOSTANDARD LVCMOS33 } [get_ports { BTN[2] }];
set_property -dict { PACKAGE_PIN B8 IOSTANDARD LVCMOS33 } [get_ports { BTN[3] }];

##===========================================================================
## 単色 LED (4個)
##===========================================================================
set_property -dict { PACKAGE_PIN H5  IOSTANDARD LVCMOS33 } [get_ports { LED[0] }];
set_property -dict { PACKAGE_PIN J5  IOSTANDARD LVCMOS33 } [get_ports { LED[1] }];
set_property -dict { PACKAGE_PIN T9  IOSTANDARD LVCMOS33 } [get_ports { LED[2] }];
set_property -dict { PACKAGE_PIN T10 IOSTANDARD LVCMOS33 } [get_ports { LED[3] }];

##===========================================================================
## RGB LED0
##===========================================================================
set_property -dict { PACKAGE_PIN G6 IOSTANDARD LVCMOS33 } [get_ports { RGB0_Red }];
set_property -dict { PACKAGE_PIN F6 IOSTANDARD LVCMOS33 } [get_ports { RGB0_Green }];
set_property -dict { PACKAGE_PIN E1 IOSTANDARD LVCMOS33 } [get_ports { RGB0_Blue }];

##===========================================================================
## RGB LED1
##===========================================================================
set_property -dict { PACKAGE_PIN G3 IOSTANDARD LVCMOS33 } [get_ports { RGB1_Red }];
set_property -dict { PACKAGE_PIN J4 IOSTANDARD LVCMOS33 } [get_ports { RGB1_Green }];
set_property -dict { PACKAGE_PIN G4 IOSTANDARD LVCMOS33 } [get_ports { RGB1_Blue }];

##===========================================================================
## RGB LED2
##===========================================================================
set_property -dict { PACKAGE_PIN J3 IOSTANDARD LVCMOS33 } [get_ports { RGB2_Red }];
set_property -dict { PACKAGE_PIN J2 IOSTANDARD LVCMOS33 } [get_ports { RGB2_Green }];
set_property -dict { PACKAGE_PIN H4 IOSTANDARD LVCMOS33 } [get_ports { RGB2_Blue }];

##===========================================================================
## RGB LED3
##===========================================================================
set_property -dict { PACKAGE_PIN K1 IOSTANDARD LVCMOS33 } [get_ports { RGB3_Red }];
set_property -dict { PACKAGE_PIN H6 IOSTANDARD LVCMOS33 } [get_ports { RGB3_Green }];
set_property -dict { PACKAGE_PIN K2 IOSTANDARD LVCMOS33 } [get_ports { RGB3_Blue }];

##===========================================================================
## USB-UART インターフェース
##===========================================================================
set_property -dict { PACKAGE_PIN D10 IOSTANDARD LVCMOS33 } [get_ports { UART_TXD }];

##===========================================================================
## Pmod Header JA (SPI接続)
## JA1 = CS, JA2 = MOSI, JA3 = MISO, JA4 = SCK
##===========================================================================
set_property -dict { PACKAGE_PIN G13 IOSTANDARD LVCMOS33 } [get_ports { JA1 }];   # CS
set_property -dict { PACKAGE_PIN B11 IOSTANDARD LVCMOS33 } [get_ports { JA2 }];   # MOSI
set_property -dict { PACKAGE_PIN A11 IOSTANDARD LVCMOS33 } [get_ports { JA3 }];   # MISO
set_property -dict { PACKAGE_PIN D12 IOSTANDARD LVCMOS33 } [get_ports { JA4 }];   # SCK

## Pmod JA 入力遅延制御（MISOのみ入力）
set_input_delay -clock sys_clk_pin -max 5.000 [get_ports {JA3}]
set_input_delay -clock sys_clk_pin -min 0.000 [get_ports {JA3}]

##===========================================================================
## タイミング制約
##===========================================================================
set_false_path -from [get_ports {BTN[*]}]
set_false_path -from [get_ports {SW[*]}]
set_false_path -to   [get_ports {UART_TXD}]
set_false_path -to   [get_ports {LED[*]}]
set_false_path -to   [get_ports {RGB*}]

## SPI出力遅延
set_output_delay -clock sys_clk_pin -max 2.000 [get_ports {JA1 JA2 JA4}]
set_output_delay -clock sys_clk_pin -min -0.500 [get_ports {JA1 JA2 JA4}]
