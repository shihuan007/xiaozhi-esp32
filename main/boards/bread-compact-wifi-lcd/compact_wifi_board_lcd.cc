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
// === 四路舵机引脚与通道定义（对应底板左下角 14, 13, 12, 11 引脚） ===
#define SERVO_HEAD_LR_PIN     GPIO_NUM_14  // 头部左右摇头
#define SERVO_HEAD_UD_PIN     GPIO_NUM_13  // 头部上下点头
#define SERVO_LEFT_HAND_PIN   GPIO_NUM_12  // 左手挥臂
#define SERVO_RIGHT_HAND_PIN  GPIO_NUM_11  // 右手挥臂

#define SERVO_CH_HEAD_LR      LEDC_CHANNEL_1
#define SERVO_CH_HEAD_UD      LEDC_CHANNEL_2
#define SERVO_CH_LEFT_HAND    LEDC_CHANNEL_3
#define SERVO_CH_RIGHT_HAND   LEDC_CHANNEL_4

// 控制指定通道的舵机转动到 angle (0~180度)
static void SetSingleServoAngle(ledc_channel_t channel, int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    uint32_t duty = 205 + (angle * (1024 - 205)) / 180;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

// 执行预设连贯动作：1=点头, 2=摇头, 3=左手挥手, 4=右手挥手, 5=双手欢呼
static void RunServoAction(int action_type) {
    switch (action_type) {
        case 1: // 点头两次
            for (int i = 0; i < 2; i++) {
                SetSingleServoAngle(SERVO_CH_HEAD_UD, 55);
                vTaskDelay(pdMS_TO_TICKS(250));
                SetSingleServoAngle(SERVO_CH_HEAD_UD, 115);
                vTaskDelay(pdMS_TO_TICKS(250));
            }
            SetSingleServoAngle(SERVO_CH_HEAD_UD, 90);
            break;
        case 2: // 左右摇头
            SetSingleServoAngle(SERVO_CH_HEAD_LR, 45);
            vTaskDelay(pdMS_TO_TICKS(300));
            SetSingleServoAngle(SERVO_CH_HEAD_LR, 135);
            vTaskDelay(pdMS_TO_TICKS(300));
            SetSingleServoAngle(SERVO_CH_HEAD_LR, 90);
            break;
        case 3: // 举左手挥一挥
            for (int i = 0; i < 2; i++) {
                SetSingleServoAngle(SERVO_CH_LEFT_HAND, 150);
                vTaskDelay(pdMS_TO_TICKS(250));
                SetSingleServoAngle(SERVO_CH_LEFT_HAND, 100);
                vTaskDelay(pdMS_TO_TICKS(250));
            }
            SetSingleServoAngle(SERVO_CH_LEFT_HAND, 90);
            break;
        case 4: // 举右手挥一挥
            for (int i = 0; i < 2; i++) {
                SetSingleServoAngle(SERVO_CH_RIGHT_HAND, 30);
                vTaskDelay(pdMS_TO_TICKS(250));
                SetSingleServoAngle(SERVO_CH_RIGHT_HAND, 80);
                vTaskDelay(pdMS_TO_TICKS(250));
            }
            SetSingleServoAngle(SERVO_CH_RIGHT_HAND, 90);
            break;
        case 5: // 双手同时举起欢呼并点头
            SetSingleServoAngle(SERVO_CH_LEFT_HAND, 150);
            SetSingleServoAngle(SERVO_CH_RIGHT_HAND, 30);
            SetSingleServoAngle(SERVO_CH_HEAD_UD, 60);
            vTaskDelay(pdMS_TO_TICKS(500));
            SetSingleServoAngle(SERVO_CH_LEFT_HAND, 90);
            SetSingleServoAngle(SERVO_CH_RIGHT_HAND, 90);
            SetSingleServoAngle(SERVO_CH_HEAD_UD, 90);
            break;
        default: // 全部归中 90 度
            SetSingleServoAngle(SERVO_CH_HEAD_LR, 90);
            SetSingleServoAngle(SERVO_CH_HEAD_UD, 90);
            SetSingleServoAngle(SERVO_CH_LEFT_HAND, 90);
            SetSingleServoAngle(SERVO_CH_RIGHT_HAND, 90);
            break;
    }
}

// 初始化四台舵机并分步归中自检（错峰上电防止电流过大重启）
static void InitAndTestServo() {
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = LEDC_TIMER_1,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    const gpio_num_t pins[4] = {
        SERVO_HEAD_LR_PIN, SERVO_HEAD_UD_PIN, SERVO_LEFT_HAND_PIN, SERVO_RIGHT_HAND_PIN
    };
    const ledc_channel_t channels[4] = {
        SERVO_CH_HEAD_LR, SERVO_CH_HEAD_UD, SERVO_CH_LEFT_HAND, SERVO_CH_RIGHT_HAND
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch_conf = {
            .gpio_num   = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = channels[i],
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = LEDC_TIMER_1,
            .duty       = 614, // 90度中心位置
            .hpoint     = 0
        };
        ledc_channel_config(&ch_conf);
        SetSingleServoAngle(channels[i], 90);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    // 开机轻轻摇头自检一次
    SetSingleServoAngle(SERVO_CH_HEAD_LR, 70);
    vTaskDelay(pdMS_TO_TICKS(250));
    SetSingleServoAngle(SERVO_CH_HEAD_LR, 110);
    vTaskDelay(pdMS_TO_TICKS(250));
    SetSingleServoAngle(SERVO_CH_HEAD_LR, 90);
}
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
  // 1. 初始化四路舵机并全部归中到 90 度
        InitAndTestServo();

        // 2. 注册四路舵机精确角度控制工具
        McpServer::GetInstance().AddTool("self.servo.set_angles",
            "同时控制机器人的四台舵机角度(0到180度，90为默认中心)。"
            "参数说明：head_lr为头部左右(30左看,150右看,90居中)；"
            "head_ud为头部上下(55低头,125抬头,90平视)；"
            "left_hand为左手(150举起,90放下)；"
            "right_hand为右手(30举起,90放下)。未提及的部位请传入90。",
            PropertyList({
                Property("head_lr", kPropertyTypeInteger, 0, 180),
                Property("head_ud", kPropertyTypeInteger, 0, 180),
                Property("left_hand", kPropertyTypeInteger, 0, 180),
                Property("right_hand", kPropertyTypeInteger, 0, 180)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                SetSingleServoAngle(SERVO_CH_HEAD_LR, properties["head_lr"].value<int>());
                SetSingleServoAngle(SERVO_CH_HEAD_UD, properties["head_ud"].value<int>());
                SetSingleServoAngle(SERVO_CH_LEFT_HAND, properties["left_hand"].value<int>());
                SetSingleServoAngle(SERVO_CH_RIGHT_HAND, properties["right_hand"].value<int>());
                return true;
            });

        // 3. 注册预设连贯动作工具（点头、摇头、挥手、欢呼）
        McpServer::GetInstance().AddTool("self.servo.do_action",
            "控制机器人做出连贯肢体动作。action_type取值："
            "1=点头，2=摇头，3=举左手挥手，4=举右手挥手，5=双手欢呼，0=全部立正归位。",
            PropertyList({
                Property("action_type", kPropertyTypeInteger, 0, 5)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                int action_type = properties["action_type"].value<int>();
                RunServoAction(action_type);
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
