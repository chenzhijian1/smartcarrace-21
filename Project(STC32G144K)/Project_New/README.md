# 引脚分配校验文档

## 判断依据

- `libraries/zf_driver/zf_driver_pwm.h`
- `libraries/zf_driver/zf_driver_encoder.h`
- `libraries/zf_driver/zf_driver_encoder.c`
- `libraries/zf_driver/zf_driver_spi.h`
- `libraries/zf_driver/zf_driver_adc.h`
- `libraries/zf_driver/zf_driver_uart.h`

## 主要更改

- 修改右电机 `DIR`引脚，避免使用互补PWM引脚。
- 修改左编码器引脚，避免与PWM输出口同组。（注意DIR与PLUSE顺序）
- 修改无线串口UART资源分组（to UART_3)。防止和烧录串口使用同组UART
- 修改电磁信号处理模块

## PWM 与编码器

| 网络名        | MCU 引脚 | 库中资源                    | 用途              | 结论                                              |
| ------------- | -------: | --------------------------- | ----------------- | ------------------------------------------------- |
| `PWMA`        |    `P60` | `PWMA_CH1P_P60`             | 风扇电机 A EN/PWM | 通过                                              |
| `PWMB`        |    `P62` | `PWMA_CH2P_P62`             | 右电机 B EN/PWM   | 通过                                              |
| `PWMC`        |    `P53` | `PWMD_CH4_P53`              | 左电机 C EN/PWM   | 通过                                              |
| `DIRA`        |    `P07` | GPIO                        | 风扇电机 A PH     | 通过                                              |
| `DIRB`        |    `P47` | GPIO / UART2_TX 复用        | 右电机 B PH       | 通过                                              |
| `DIRC`        |    `P06` | GPIO / TIM4 / ADC2_CH6 复用 | 左电机 C PH       | 通过                                              |
| `SPEEDLDIR`   |    `P75` | `PWMB_CH2_P75`              | 左编码器方向      | 资源通过，顺序需确认                              |
| `SPEEDLPLUSE` |    `P74` | `PWMB_CH1_P74`              | 左编码器脉冲      | 资源通过，顺序需确认                              |
| `SPEEDRDIR`   |    `P72` | `PWME_CH2P_P72`             | 右编码器方向      | 资源通过，顺序需确认                              |
| `SPEEDRPLUSE` |    `P70` | `PWME_CH1P_P70`             | 右编码器脉冲      | 资源通过，顺序需确认                              |
| `NSLEEP`      |    `P36` | GPIO                        | 驱动芯片唤醒      | 三驱动共用一个唤醒引脚，开机后给20us~40us低位脉冲 |

## 电磁信号处理模块（主板丝印已标注IN1~4输入引脚）

| 网络名   | MCU引脚 | 对应功能                                                                                 |
| -------- | ------- | ---------------------------------------------------------------------------------------- |
| `OE1`    | `P04`   | GPIO，模拟门1输出使能。高电平时该模拟门输出禁用，低电平时正常输出                        |
| `MUX1`   | `P03`   | GPIO，信号选择脚。低电平输出 `IN2`通道值，高电平输出 `IN1`通道值                         |
| `OE2`    | `P02`   | GPIO，模拟门2输出使能。高电平时该模拟门输出禁用，低电平时正常输出                        |
| `MUX2`   | `P01`   | GPIO，信号选择脚。低电平输出 `IN4`通道值，高电平输出 `IN3`通道值                         |
| `ROE`    | `P00`   | GPIO，倍率调节模拟门使能。高电平时该模拟门输出禁用，低电平时正常输出（请默认低电平）     |
| `RMUX`   | `P46`   | GPIO，程控倍率选择。低电平对应 `31.7X`放大倍率，高电平对应 `20.7X`放大倍率。不可高频调节 |
| `ADC_IN` | `P10`   | ADC 输入，`ADC1_CH0_P10`                                                                 |
| `ADC_IN` | `P11`   | GPIO,用于电荷泄放（低电平泄放电荷，初始化为高电平）                                      |

## 无线串口

| 网络名 | MCU引脚 | 对应功能       |
| ------ | ------- | -------------- |
| `WTX`  | `P51`   | `UART3_TX_P51` |
| `WRX`  | `P50`   | `UART3_RX_P50` |
| `RTS`  | `P42`   | GPIO           |
| `CMD`  | `P37`   | GPIO           |

## 总引脚-网络名-功能对应表

| 网络名   |  MCU 引脚 | 对应功能                                              |
| -------- | --------: | ----------------------------------------------------- |
| `PWMC`   |     `P53` | 电机 C PWM，`PWMD_CH4_P53`                            |
| `DIRC`   |     `P06` | 电机 C 方向，GPIO                                     |
| `DIRA`   |     `P07` | 电机 A 方向，GPIO                                     |
| `PWMA`   |     `P60` | 电机 A PWM，`PWMA_CH1P_P60`                           |
| `PWMB`   |     `P62` | 电机 B PWM，`PWMA_CH2P_P62`                           |
| `ADC_IN` | `P10 P11` | ADC 输入，`ADC1_CH0_P10`,`IO_P11`用于电荷泄放         |
| `DIRB`   |     `P47` | 电机 B 方向，GPIO                                     |
| `RST`    | `P54/RST` | 复位                                                  |
| `VREF+`  |   `VREF+` | ADC 参考电压                                          |
| `CS2`    |     `P64` | 陀螺仪 SPI 片选，GPIO                                 |
| `MOSI2`  |     `P65` | 陀螺仪 MOSI，`SPI2_CH1_MOSI_P65`                      |
| `MISO2`  |     `P66` | 陀螺仪 MISO，`SPI2_CH1_MISO_P66`                      |
| `SCK2`   |     `P67` | 陀螺仪 SCLK，`SPI2_CH1_SCLK_P67`                      |
| `RX1`    |     `P30` | 烧录串口 1 RX，`UART1_RX_P30`                         |
| `TX1`    |     `P31` | 烧录串口 1 TX，`UART1_TX_P31`                         |
| `LED`    |     `P52` | LED GPIO                                              |
| `OE1`    |     `P04` | 外设使能 GPIO                                         |
| `MUX1`   |     `P03` | MUX 选择 GPIO                                         |
| `OE2`    |     `P02` | 外设使能 GPIO                                         |
| `MUX2`   |     `P01` | MUX 选择 GPIO                                         |
| `ROE`    |     `P00` | 外设使能 GPIO                                         |
| `RMUX`   |     `P46` | MUX 选择 GPIO                                         |
| `KEY1`   |     `P77` | 按键输入                                              |
| `KEY2`   |     `P76` | 按键输入                                              |
| `KEY3`   |     `P27` | 按键输入                                              |
| `KEY4`   |     `P45` | 按键输入；也可作 SPI1/3 MISO，但当前已占用            |
| `DBL`    |     `P26` | 屏幕背光/控制 GPIO                                    |
| `DSCLK`  |     `P25` | 屏幕 SCLK，`SPI1_CH2_SCLK_P25` 或 `SPI3_CH1_SCLK_P25` |
| `DMOSI`  |     `P23` | 屏幕 MOSI，`SPI1_CH2_MOSI_P23` 或 `SPI3_CH1_MOSI_P23` |
| `DCS`    |     `P22` | 屏幕片选 GPIO                                         |
| `DRST`   |     `P21` | 屏幕复位 GPIO                                         |
| `DC`     |     `P20` | 屏幕数据/命令 GPIO                                    |
| `WTX`    |     `P51` | UART3_TX 复用；无线串口命名视角需确认                 |
| `WRX`    |     `P50` | UART3_RX 复用；无线串口命名视角需确认                 |
| `RTS`    |     `P42` | 无线串口 RTS GPIO                                     |
| `CMD`    |     `P37` | 外设控制 GPIO                                         |
| `NSLEEP` |     `P36` | DRV8245 休眠控制 GPIO                                 |
