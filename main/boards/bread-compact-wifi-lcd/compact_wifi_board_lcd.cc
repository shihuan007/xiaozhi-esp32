#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cmath>
#include "mcp_server.h"
// ==================== 【位置二】四舵机纯锂电池【分时错峰+软启动+到位断电】模块 ====================
// 1~4号舵机严格保留你原有的 14, 13, 12, 11 引脚定义；触摸与红外分配至上方空闲的 10, 9, 8 引脚
#define SERVO_PAN_GPIO        GPIO_NUM_14   // 1号舵机: 头部左右摇头 (head_lr)
#define SERVO_TILT_GPIO       GPIO_NUM_13   // 2号舵机: 头部上下点头 (head_ud)
#define SERVO_LEFT_ARM_GPIO   GPIO_NUM_12   // 3号舵机: 左手挥臂 (left_hand)
#define SERVO_RIGHT_ARM_GPIO  GPIO_NUM_11   // 4号舵机: 右手挥臂 (right_hand)

#define SENSOR_TOUCH_GPIO     GPIO_NUM_10   // TTP223 触摸模块 (高电平触发)
#define SENSOR_IR_LEFT_GPIO   GPIO_NUM_9    // 左侧红外避障模块 (低电平触发)
#define SENSOR_IR_RIGHT_GPIO  GPIO_NUM_8    // 右侧红外避障模块 (低电平触发)

#define SERVO_TOTAL_NUM       4
#define SERVO_STEP_MS         20            // 每 20ms 微步更新一次 (软启动限制瞬态电流)
#define SERVO_MAX_STEP_DEG    2             // 每步最大转动 2 度
#define SERVO_HOLD_TICKS      6             // 到位保持 120ms 后自动切断 PWM 脉冲省电
#define SERVO_COOLDOWN_TICKS  3             // 两个舵机切换间歇 60ms 供板载5V电容回充

struct EspIdfStaggeredServo {
    gpio_num_t pin;
    ledc_channel_t channel;
    int current_angle;
    int target_angle;
    int min_angle;
    int max_angle;
    int hold_counter;
    bool is_attached;
};

static EspIdfStaggeredServo g_servos[SERVO_TOTAL_NUM] = {
    {SERVO_PAN_GPIO,       LEDC_CHANNEL_1, 90, 90, 30, 150, 0, false}, // 0: 头部左右
    {SERVO_TILT_GPIO,      LEDC_CHANNEL_2, 90, 90, 55, 125, 0, false}, // 1: 头部上下
    {SERVO_LEFT_ARM_GPIO,  LEDC_CHANNEL_3, 90, 90, 20, 160, 0, false}, // 2: 左手
    {SERVO_RIGHT_ARM_GPIO, LEDC_CHANNEL_4, 90, 90, 20, 160, 0, false}  // 3: 右手
};

static uint32_t ServoAngleToDuty(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    uint32_t pulse_us = 500 + (angle * 2000) / 180;
    return (pulse_us * 16383) / 20000;
}

static void ServoWriteAngle(int idx, int angle) {
    uint32_t duty = ServoAngleToDuty(angle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, g_servos[idx].channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, g_servos[idx].channel);
    g_servos[idx].is_attached = true;
}

static void ServoDetach(int idx) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, g_servos[idx].channel, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, g_servos[idx].channel);
    g_servos[idx].is_attached = false;
}

static void SetServoTargetSafe(int idx, int target) {
    if (idx < 0 || idx >= SERVO_TOTAL_NUM) return;
    if (target < g_servos[idx].min_angle) target = g_servos[idx].min_angle;
    if (target > g_servos[idx].max_angle) target = g_servos[idx].max_angle;
    g_servos[idx].target_angle = target;
    g_servos[idx].hold_counter = 0;
}

static void StaggeredServoTask(void* arg) {
    int active_idx = -1;
    int cooldown_ticks = 0;
    int auto_reset_ticks = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(SERVO_STEP_MS));

        if (active_idx == -1) {
            if (cooldown_ticks > 0) {
                cooldown_ticks--;
                continue;
            }
            for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
                if (g_servos[i].current_angle != g_servos[i].target_angle) {
                    active_idx = i;
                    g_servos[i].hold_counter = 0;
                    break;
                }
            }
        }

        if (active_idx != -1) {
            EspIdfStaggeredServo &s = g_servos[active_idx];
            if (s.current_angle != s.target_angle) {
                int diff = s.target_angle - s.current_angle;
                if (std::abs(diff) <= SERVO_MAX_STEP_DEG) {
                    s.current_angle = s.target_angle;
                } else {
                    s.current_angle += (diff > 0) ? SERVO_MAX_STEP_DEG : -SERVO_MAX_STEP_DEG;
                }
                ServoWriteAngle(active_idx, s.current_angle);
            } else {
                s.hold_counter++;
                if (s.hold_counter >= SERVO_HOLD_TICKS) {
                    ServoDetach(active_idx);
                    active_idx = -1;
                    cooldown_ticks = SERVO_COOLDOWN_TICKS;
                }
            }
            continue;
        }

        if (auto_reset_ticks > 0) {
            auto_reset_ticks--;
            if (auto_reset_ticks == 0) {
                for (int i = 0; i < SERVO_TOTAL_NUM; i++) SetServoTargetSafe(i, 90);
            }
        } else {
            if (gpio_get_level(SENSOR_TOUCH_GPIO) == 1) {
                SetServoTargetSafe(1, 65);
                SetServoTargetSafe(2, 145);
                SetServoTargetSafe(3, 35);
                auto_reset_ticks = 60;
            } else if (gpio_get_level(SENSOR_IR_LEFT_GPIO) == 0) {
                SetServoTargetSafe(0, 135);
                SetServoTargetSafe(2, 150);
                auto_reset_ticks = 75;
            } else if (gpio_get_level(SENSOR_IR_RIGHT_GPIO) == 0) {
                SetServoTargetSafe(0, 45);
                SetServoTargetSafe(3, 30);
                auto_reset_ticks = 75;
            }
        }
    }
}

static void InitStaggeredServosAndSensors() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << SENSOR_TOUCH_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << SENSOR_IR_LEFT_GPIO) | (1ULL << SENSOR_IR_RIGHT_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_conf.duty_resolution = LEDC_TIMER_14_BIT;
    timer_conf.timer_num = LEDC_TIMER_1;
    timer_conf.freq_hz = 50;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
        ledc_channel_config_t ch_conf = {};
        ch_conf.gpio_num = g_servos[i].pin;
        ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_conf.channel = g_servos[i].channel;
        ch_conf.timer_sel = LEDC_TIMER_1;
        ch_conf.duty = 0;
        ch_conf.hpoint = 0;
        ledc_channel_config(&ch_conf);

        ServoWriteAngle(i, 90);
        vTaskDelay(pdMS_TO_TICKS(220));
        ServoDetach(i);
        vTaskDelay(pdMS_TO_TICKS(60));
    }

    McpServer::GetInstance().AddTool("self.servo.set_angles",
        "分时错峰控制机器人的四台舵机角度(0到180度, 90为默认中心)。"
        "参数说明: head_lr为头部左右(30左看,150右看,90居中)；"
        "head_ud为头部上下(55低头,125抬头,90平视)；"
        "left_hand为左手(150举起,90放下)；"
        "right_hand为右手(30举起,90放下)。未提及的部位请传入90。",
        PropertyList({
            Property("head_lr", kPropertyTypeInteger, 90, 30, 150),
            Property("head_ud", kPropertyTypeInteger, 90, 55, 125),
            Property("left_hand", kPropertyTypeInteger, 90, 20, 160),
            Property("right_hand", kPropertyTypeInteger, 90, 20, 160)
        }),
        [](const PropertyList& properties) -> ReturnValue {
            SetServoTargetSafe(0, properties["head_lr"].value<int>());
            SetServoTargetSafe(1, properties["head_ud"].value<int>());
            SetServoTargetSafe(2, properties["left_hand"].value<int>());
            SetServoTargetSafe(3, properties["right_hand"].value<int>());
            return true;
        });

    xTaskCreate(StaggeredServoTask, "servo_stagger", 4096, nullptr, 5, nullptr);
}
// =========================================================================================
#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "CompactWifiBoardLCD"

class CompactWifiBoardLCD : public WifiBoard {
private:
 
    Button boot_button_;
    LcdDisplay* display_;

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        
        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
       display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeTools() {
        static LampController lamp(LAMP_GPIO);
    }

public:
    CompactWifiBoardLCD() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeTools();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }

        // 1. 启动四舵机分时错峰控制、self.servo.set_angles工具与触摸/红外检测
        InitStaggeredServosAndSensors();

        // 2. 注册预设连贯动作工具（点头、摇头、挥手、欢呼 - 纯电池分时错峰版）
        McpServer::GetInstance().AddTool("self.servo.do_action",
            "控制机器人做出连贯肢体动作。action_type取值："
            "1=点头, 2=摇头, 3=举左手挥手, 4=举右手挥手, 5=双手欢呼, 0=全部立正归位。",
            PropertyList({
                Property("action_type", kPropertyTypeInteger, 0, 5)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                int action_type = properties["action_type"].value<int>();
                switch (action_type) {
                    case 1: // 点头 (仅动2号头部上下舵机)
                        SetServoTargetSafe(1, 60);
                        break;
                    case 2: // 摇头 (仅动1号头部左右舵机)
                        SetServoTargetSafe(0, 135);
                        break;
                    case 3: // 举左手挥手 (仅动3号左手舵机)
                        SetServoTargetSafe(2, 150);
                        break;
                    case 4: // 举右手挥手 (仅动4号右手舵机)
                        SetServoTargetSafe(3, 30);
                        break;
                    case 5: // 双手欢呼 (后台错峰调度器会自动排队：先抬头 -> 再举左手 -> 再举右手)
                        SetServoTargetSafe(1, 65);
                        SetServoTargetSafe(2, 150);
                        SetServoTargetSafe(3, 30);
                        break;
                    case 0: // 全部立正归位 (依次排队回中90度)
                    default:
                        SetServoTargetSafe(0, 90);
                        SetServoTargetSafe(1, 90);
                        SetServoTargetSafe(2, 90);
                        SetServoTargetSafe(3, 90);
                        break;
                }
                return true;
            });
    }
    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(CompactWifiBoardLCD);
