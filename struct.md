```mermaid
graph TD
    tim[TIM中断]

    subgraph "获取输入"
        adc_control[ADC控制器]
        adcs[ADC输入源 * 4]

        tim --"1.激活"--> adc_control
        adcs --"2.读取"--> adc_control
    end

    subgraph "缓冲区"
        temp[<b>临时缓冲区 * 4</b><br>暂时存放ADC读取到的数据]
        ring[<b>环形缓冲区 * 4</b><br>供滤波器访问]

        adc_control --"3.写入"--> temp
        temp --"4.写入"--> ring
    end

    subgraph "信号处理区"
        denoise[<b>滤波器</b><br>去除电路底噪，提升SNR]
        judge[裁判]

        ring --"5.读取"--> denoise
        denoise --"6.返回降噪后的数据"--> judge
    end

    subgraph "输出区"
        output[输出控制器]
        led[LED * 8]
        oled_driver[OLED驱动]
        oled[OLED显示屏]
        motor_circuit[电机驱动电路]
        motor[电机]

        judge --"7.解算位置<br>8.发送位置"--> output
        output --"9.点亮对应LED"--> led
        output --"9.控制"--> motor_circuit
        motor_circuit --"驱动"--> motor
        output --"9.发送距离信息"--> oled_driver
        oled_driver --"绘制并显示"--> oled
    end
```

上面就是软件层面工作的流程图了

## 引脚占用

### SSD1306

需要一个I2C_SCL一个I2C_SCA。检查了一下DataSheet，AI是对的，所以占用下面几个引脚：

PB6: I2C1_SCL
PB7: I2C1_SCA

### 74HC138

只占用三个引脚，负责输出数据：
PA8、PA9、PA10

### MAX9814

必须占用ADC引脚，四个OUT选定为:

PA0
PA1
PA2
PA4

### TB6612

AIN1: PB0
AIN2: PB1
PWMA: PA3
