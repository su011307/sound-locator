#include <cmath>

#include "tdoa.hpp"
#include "utils.hpp"
#include "driver.hpp"
#include "i2c.hpp"

#include "motor.hpp"
#include "led.hpp"
#include "mic.hpp"
#include "oled.hpp"

// @warning 对于ADC通道的采样延迟，务必查阅手册并予以补偿
// 暂时不清楚ADC通道切换所占用的时钟周期。不过可以肯定这个时长是固定、可预测的

// 全局的外设句柄定义
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim_adc;
TIM_HandleTypeDef htim_motor;
TIM_HandleTypeDef htim_encoder;
I2C_HandleTypeDef hi2c1;

MicrophoneMatrix *g_mic_matrix = nullptr;

void __System_Clock_Config();

// @brief 初始化外设所需要的时钟
void __Enable_Clocks()
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
}

// @brief 初始化ADC和DMA
void __Init_ADC(
    ADC_HandleTypeDef *hadc,
    DMA_HandleTypeDef *hdma,
    ADC_TypeDef *ADC_INSTANCE,
    DMA_Stream_TypeDef *DMA_STREAM,
    const std::array<uint32_t, MIC_NUMBER> &ADC_CHANNELS,
    uint32_t DMA_CHANNEL)
{
    // 配置ADC
    hadc->Instance = ADC_INSTANCE;
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    hadc->Init.ScanConvMode = ENABLE;
    hadc->Init.ContinuousConvMode = DISABLE;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc->Init.NbrOfConversion = MIC_NUMBER;

    HAL_ADC_Init(hadc);

    // 配置ADC通道
    ADC_ChannelConfTypeDef config = {0};
    config.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    config.Channel = ADC_CHANNELS[0];
    config.Rank = 1;
    HAL_ADC_ConfigChannel(hadc, &config);

    config.Channel = ADC_CHANNELS[1];
    config.Rank = 2;
    HAL_ADC_ConfigChannel(hadc, &config);

    config.Channel = ADC_CHANNELS[2];
    config.Rank = 3;
    HAL_ADC_ConfigChannel(hadc, &config);

    config.Channel = ADC_CHANNELS[3];
    config.Rank = 4;
    HAL_ADC_ConfigChannel(hadc, &config);

    // 配置DMA
    hdma->Instance = DMA_STREAM;
    hdma->Init.Channel = DMA_CHANNEL;
    hdma->Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma->Init.PeriphInc = DMA_MINC_DISABLE;
    hdma->Init.MemInc = DMA_MINC_ENABLE;
    hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma->Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma->Init.Mode = DMA_CIRCULAR;
    hdma->Init.Priority = DMA_PRIORITY_HIGH;
    hdma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(hdma);

    __HAL_LINKDMA(hadc, DMA_Handle, *hdma);  // @warning [DISCLAIMER] 这里查不到文档，AI说填DMA_Handle，我也不是很确定
}

void __Init_ADC_Timer(TIM_HandleTypeDef *htim, TIM_TypeDef *TIM_INSTANCE)
{
    htim->Instance = TIM_INSTANCE;
    htim->Init.Prescaler = 9;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 189;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(htim);

    TIM_MasterConfigTypeDef config = {0};
    config.MasterOutputTrigger = TIM_TRGO_UPDATE;
    config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(htim, &config);
}

void __Init_Mic_GPIO(
    const std::array<uint16_t, MIC_NUMBER> &GPIO_PINS)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = [GPIO_PINS]
    {
        uint16_t pin_result = 0;
        for (const auto &pin : GPIO_PINS)
        {
            pin_result |= pin;
        }
        return pin_result;
    }();
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

LEDMatrix *__Init_LEDs(
    GPIO_TypeDef *GPIO_PORT,
    const std::array<uint16_t, 4> &GPIO_PINS)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = [GPIO_PINS]
    {
        uint32_t pin_result = 0;
        for (const auto &pin : GPIO_PINS)
        {
            pin_result |= pin;
        }
        return pin_result;
    }();
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIO_PORT, &gpio);

    static GPIO a0(GPIO_PORT, GPIO_PINS[0]);
    static GPIO a1(GPIO_PORT, GPIO_PINS[1]);
    static GPIO a2(GPIO_PORT, GPIO_PINS[2]);
    static GPIO e1(GPIO_PORT, GPIO_PINS[3]);

    return new LEDMatrix(&a0, &a1, &a2, &e1);
}

OLED *__Init_Screen(
    GPIO_TypeDef *GPIO_PORT,
    uint8_t GPIO_ALTERNATE,
    I2C_HandleTypeDef *hi2c,
    I2C_TypeDef *I2C_INSTANCE,
    const std::pair<uint16_t, uint16_t> &GPIO_PINS)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PINS.first | GPIO_PINS.second;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_ALTERNATE;
    HAL_GPIO_Init(GPIO_PORT, &gpio);

    hi2c->Instance = I2C_INSTANCE;
    hi2c->Init.ClockSpeed = 400000;
    hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(hi2c);

    static I2CPort i2c_port(hi2c);
    OLED *oled = new OLED(i2c_port);

    if (!oled->init())
    {
        // ERROR_HANDLER()
    }
    return oled;
}

Motor *__Init_Motor(
    GPIO_TypeDef *GPIO_CONTROLL_PORT,
    GPIO_TypeDef *GPIO_PWM_PORT,
    const std::pair<GPIO_TypeDef *, GPIO_TypeDef *> &GPIO_ENCODER_PORTS,
    uint8_t GPIO_PWM_ALTERNATE,
    uint8_t GPIO_ENCODER_ALTERNATE,
    const std::pair<uint16_t, uint16_t> &GPIO_CONTROL_PINS,
    const std::pair<uint16_t, uint16_t> &GPIO_ENCODER_PINS,
    uint16_t GPIO_PWM_PIN,
    TIM_HandleTypeDef *htim_pwm,
    TIM_HandleTypeDef *htim_encoder,
    TIM_TypeDef *TIM_PWM_INSTANCE,
    TIM_TypeDef *TIM_ENCODER_INSTANCE,
    uint32_t TIM_PWM_CHANNEL)
{
    // 初始化控制的两个引脚
    GPIO_InitTypeDef ctrl_gpio = {0};
    ctrl_gpio.Pin = GPIO_CONTROL_PINS.first | GPIO_CONTROL_PINS.second;
    ctrl_gpio.Mode = GPIO_MODE_OUTPUT_PP;
    ctrl_gpio.Pull = GPIO_NOPULL;
    ctrl_gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIO_CONTROLL_PORT, &ctrl_gpio);

    /*
     * 初始化负责PWM的引脚和定时器
     * Timer参考文档参见 *UM1725* pp.1059-1060 (68.1.2)
     */
    GPIO_InitTypeDef pwm_gpio = {0};
    pwm_gpio.Pin = GPIO_PWM_PIN;
    pwm_gpio.Mode = GPIO_MODE_AF_PP;
    pwm_gpio.Alternate = GPIO_PWM_ALTERNATE;
    HAL_GPIO_Init(GPIO_PWM_PORT, &pwm_gpio);

    htim_pwm->Instance = TIM_PWM_INSTANCE;
    htim_pwm->Init.Prescaler = 9;  // @remark Prescaler和Period是按照AHB2总线（50MHz）计算的。这还需要考虑时钟的预分频系数
    htim_pwm->Init.Period = 2499;  // 尽可能大的Period可以让PWM分辨率更高
    htim_pwm->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_pwm->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(htim_pwm);

    TIM_OC_InitTypeDef config = {0};
    config.OCMode = TIM_OCMODE_PWM1;  // 计数小于Pulse时输出高
    config.Pulse = 0;  // 写入CCR寄存器的值，位于0x0000-0xFFFF之间。这个值决定了占空比
    config.OCPolarity = TIM_OCPOLARITY_HIGH;  // 输出极性
    config.OCFastMode = TIM_OCFAST_DISABLE;  // 决定是否启用Fast mode状态（？什么鬼，不知道是干什么的）
    HAL_TIM_PWM_ConfigChannel(htim_pwm, &config, TIM_PWM_CHANNEL);

    // 初始化编码器部分
    GPIO_InitTypeDef encoder_gpio = {0};
    encoder_gpio.Pin = GPIO_ENCODER_PINS.first;  // 配置编码器的A相
    encoder_gpio.Mode = GPIO_MODE_AF_PP;
    encoder_gpio.Pull = GPIO_PULLUP;
    encoder_gpio.Alternate = GPIO_ENCODER_ALTERNATE;  // 决定使用哪个复用外设功能。这不是定义引脚，而是定义复用功能
    HAL_GPIO_Init(GPIO_ENCODER_PORTS.first, &encoder_gpio);

    encoder_gpio.Pin = GPIO_ENCODER_PINS.second;  // 配置编码器的B相
    HAL_GPIO_Init(GPIO_ENCODER_PORTS.second, &encoder_gpio);

    // 为编码器配置一个Timer
    htim_encoder->Instance = TIM_ENCODER_INSTANCE;
    htim_encoder->Init.Prescaler = 1;  // 没有Prescaler，用来统计GMR编码器发出的原始脉冲数量
    htim_encoder->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_encoder->Init.Period = 0xFFFFFFFFU;  // @remark 具体的值取决于计数器是32位还是16位。
    htim_encoder->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    
    /*
     * 配置供编码器使用的Timer，参阅*UM1725* pp.1061-1062 (68.1.5)
     */
    TIM_Encoder_InitTypeDef encoder_config = {0};
    encoder_config.EncoderMode = TIM_ENCODERMODE_TI12;  // 同时参考正交编码器的A、B相。一个周期发出4个计数
    encoder_config.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC1Selection = TIM_ICSELECTION_DIRECTTI;  // ICx <-> TIx一一对应
    encoder_config.IC1Prescaler = TIM_ICPSC_DIV1;  // 预分频器。每检测到一个edge都会带来一个Capture
    encoder_config.IC1Filter = 3;  // @remark 这里对编码器输出一点轻微的滤波。如果计数值乱跳，就需要加入一定程度的滤波了
    
    // TIM_Encoder有两个相，这里配置B相
    encoder_config.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC1Filter = 3;

    HAL_TIM_Encoder_Init(htim_encoder, &encoder_config);  // 提交编码器的配置
    HAL_TIM_Encoder_Start(htim_encoder, TIM_CHANNEL_ALL);  // 启动编码器接口

    // 创建对应的抽象实例
    static GPIO ain1(GPIO_CONTROLL_PORT, GPIO_CONTROL_PINS.first);
    static GPIO ain2(GPIO_CONTROLL_PORT, GPIO_CONTROL_PINS.second);
    static PIDParams pid = {.Kp = 1.2f, .Ki = 0.5f, .Kd = 0.2f, .intergral = 0.0f, .diveritive = 0.0f, .last_error = 0.0f};
    static Encoder encoder(htim_encoder, DELTA);

    return new Motor(htim_pwm, TIM_PWM_CHANNEL, &ain1, &ain2, &encoder, &pid);
}

int main(){
    // 全局初始化
    HAL_Init();
    __System_Clock_Config();
    __Enable_Clocks();

    // 模块初始化
    const std::array<uint16_t, MIC_NUMBER> mic_pins = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3,};
    const std::array<uint32_t, MIC_NUMBER> mic_channels = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3};

    __Init_Mic_GPIO(mic_pins);
    __Init_ADC_Timer(&htim_adc, TIM3);
    __Init_ADC(&hadc1, &hdma_adc1, ADC1, DMA2_Stream0, mic_channels, DMA_CHANNEL_0);
    g_mic_matrix = new MicrophoneMatrix(&hadc1, &htim_adc);

    const std::array<uint16_t, 4> led_pins = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};
    LEDMatrix *leds = __Init_LEDs(GPIOB, led_pins);

    OLED *oled = __Init_Screen(GPIOB, GPIO_AF4_I2C1, &hi2c1, I2C1, {GPIO_PIN_6, GPIO_PIN_7});

    Motor *motor = __Init_Motor(
        GPIOC, GPIOC, {GPIOA, GPIOB}, 
        GPIO_AF2_TIM4, GPIO_AF1_TIM2,
        {GPIO_PIN_0, GPIO_PIN_1}, {GPIO_PIN_15, GPIO_PIN_3}, GPIO_PIN_6,
        &htim_motor, &htim_encoder, TIM4, TIM2, TIM_CHANNEL_1
    );

    leds->init();
    oled->clear();
    oled->write("42");
    oled->refresh();
    g_mic_matrix->start();

    // 麦克风的物理位置，单位：cm
    const std::array<Point, MIC_NUMBER> mic_locations = {{
        {4.75, 4.75}, {4.75, -4.75},
        {-4.75, 4.75}, {-4.75, -4.75}
    }};

    while(1){
        Mode curr_mode = g_mic_matrix->get_mode();

        switch (curr_mode){
            case Mode::Single:
            case Mode::Random:
            {
                if (g_mic_matrix->is_ready())
                {
                    oled->clear();
                    oled->write("...");
                    oled->refresh();
                    auto timestamps = g_mic_matrix->get_timestamps();
                    std::optional<Point> position = compute_position(mic_locations, timestamps);

                    if (position.has_value())
                    {
                        auto [x, y] = *position;

                        float angle = atan(y/x) * 180.0f / 3.1415927f;
                        if (angle < 0){
                            angle += 360.0f;
                        }

                        motor->abs_update(angle);
                        leds->light_up(static_cast<uint16_t>(angle * 10.0f));
                    }
                    else
                    {

                    }
                }
                break;
            }

            case Mode::Continuous:
            {
                break;
            }

            case Mode::Measure:
            {
                break;
            }
        }

        HAL_Delay(10);
    }
    
    return 0;
}

// @remark 回调函数不支持传入其它参数，如果需要修改，要把这里也改了
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    if (hadc->Instance == ADC1){
        if (g_mic_matrix != nullptr){
            g_mic_matrix->callback();
        }
    }
}

void __System_Clock_Config(){
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;  // 选用HSE @8MHz作Oscillator
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;  // 启动HSE

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;  // 启动PLL(Phase-Locked Loop)，用来放大频率
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;  // 选择HSE @8MHz作为PLL的输入源

    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 200;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;  // 分频后频率 f = (f_HSE / PLLM) * PLLN / PLLP
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // AHB @100MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_SYSCLK_DIV2;  // APB1 @50MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_SYSCLK_DIV1;  // APB2 @100MHz

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}