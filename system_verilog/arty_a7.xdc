## arty_a7.xdc
## Arty A7-35T / A7-100T 用制約ファイル
## 対応ボードリビジョン: Rev. D / Rev. E
## Digilent 公式 XDC に準拠

##===========================================================================
## コンフィグ設定
##===========================================================================
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]

##===========================================================================
## クロック信号 (100MHz)
##===========================================================================
set_property -dict { PACKAGE_PIN E3 IOSTANDARD LVCMOS33 } [get_ports { CLK100MHZ }];
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports {CLK100MHZ}];

##===========================================================================
## スライドスイッチ (4個、プルアップ済み)
##===========================================================================
set_property -dict { PACKAGE_PIN A8  IOSTANDARD LVCMOS33 } [get_ports { SW[0] }];
set_property -dict { PACKAGE_PIN C11 IOSTANDARD LVCMOS33 } [get_ports { SW[1] }];
set_property -dict { PACKAGE_PIN C10 IOSTANDARD LVCMOS33 } [get_ports { SW[2] }];
set_property -dict { PACKAGE_PIN A10 IOSTANDARD LVCMOS33 } [get_ports { SW[3] }];

##===========================================================================
## プッシュボタン (4個、負論理: 押すとLOW、プルアップ済み)
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
## RGB LED0 (Red=G6, Green=F6, Blue=E1)
##===========================================================================
set_property -dict { PACKAGE_PIN G6 IOSTANDARD LVCMOS33 } [get_ports { RGB0_Red }];
set_property -dict { PACKAGE_PIN F6 IOSTANDARD LVCMOS33 } [get_ports { RGB0_Green }];
set_property -dict { PACKAGE_PIN E1 IOSTANDARD LVCMOS33 } [get_ports { RGB0_Blue }];

##===========================================================================
## RGB LED1 (Red=G3, Green=J4, Blue=G4)
##===========================================================================
set_property -dict { PACKAGE_PIN G3 IOSTANDARD LVCMOS33 } [get_ports { RGB1_Red }];
set_property -dict { PACKAGE_PIN J4 IOSTANDARD LVCMOS33 } [get_ports { RGB1_Green }];
set_property -dict { PACKAGE_PIN G4 IOSTANDARD LVCMOS33 } [get_ports { RGB1_Blue }];

##===========================================================================
## RGB LED2 (Red=J3, Green=J2, Blue=H4)
##===========================================================================
set_property -dict { PACKAGE_PIN J3 IOSTANDARD LVCMOS33 } [get_ports { RGB2_Red }];
set_property -dict { PACKAGE_PIN J2 IOSTANDARD LVCMOS33 } [get_ports { RGB2_Green }];
set_property -dict { PACKAGE_PIN H4 IOSTANDARD LVCMOS33 } [get_ports { RGB2_Blue }];

##===========================================================================
## RGB LED3 (Red=K1, Green=H6, Blue=K2)
##===========================================================================
set_property -dict { PACKAGE_PIN K1 IOSTANDARD LVCMOS33 } [get_ports { RGB3_Red }];
set_property -dict { PACKAGE_PIN H6 IOSTANDARD LVCMOS33 } [get_ports { RGB3_Green }];
set_property -dict { PACKAGE_PIN K2 IOSTANDARD LVCMOS33 } [get_ports { RGB3_Blue }];

##===========================================================================
## USB-UART インターフェース (TXD=D10)
##===========================================================================
set_property -dict { PACKAGE_PIN D10 IOSTANDARD LVCMOS33 } [get_ports { UART_TXD }];

##===========================================================================
## タイミング制約
##===========================================================================
## ボタン・スイッチ入力非同期なので false path を設定
set_false_path -from [get_ports {BTN[*]}]
set_false_path -from [get_ports {SW[*]}]
set_false_path -to   [get_ports {UART_TXD}]

##===========================================================================
## ドライブ強度設定 (RGB LED用)
##===========================================================================
set_property DRIVE 4 [get_ports {RGB*_Red RGB*_Green RGB*_Blue}];
set_property SLEW SLOW [get_ports {RGB*_Red RGB*_Green RGB*_Blue}];
