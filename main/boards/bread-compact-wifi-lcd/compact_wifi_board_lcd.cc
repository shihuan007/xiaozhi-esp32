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
// ==================== 【位置二】4舵机错峰 + 5/6号双齿轮履带【同步驱动】模块 ====================
#define SERVO_PAN_GPIO         GPIO_NUM_14   // 1号舵机: 头部左右摇头 (head_lr)
#define SERVO_TILT_GPIO        GPIO_NUM_13   // 2号舵机: 头部上下点头 (head_ud)
#define SERVO_LEFT_ARM_GPIO    GPIO_NUM_12   // 3号舵机: 左手挥臂 (left_hand)
#define SERVO_RIGHT_ARM_GPIO   GPIO_NUM_11   // 4号舵机: 右手挥臂 (right_hand)
#define SERVO_LEFT_GEAR_GPIO   GPIO_NUM_10   // 5号舵机: 左侧齿轮履带 (left_gear)
#define SERVO_RIGHT_GEAR_GPIO  GPIO_NUM_9    // 6号舵机: 右侧齿轮履带 (right_gear)

#define SENSOR_TOUCH_GPIO      GPIO_NUM_8    // TTP223 触摸模块 (高电平触发)
#define SENSOR_IR_LEFT_GPIO    GPIO_NUM_17   // 左侧红外模块 (借用空置4G模块引脚, 低电平触发)
#define SENSOR_IR_RIGHT_GPIO   GPIO_NUM_18   // 右侧红外模块 (借用空置4G模块引脚, 低电平触发)

#define SERVO_UPPER_NUM        4             // 上半身错峰舵机数量 (索引 0~3)
#define SERVO_TOTAL_NUM        6             // 总舵机数量 (索引 4, 5 为底部同步双齿轮)
#define GEAR_LEFT_IDX          4             // 5号左齿轮舵机索引
#define GEAR_RIGHT_IDX         5             // 6号右齿轮舵机索引
#define GEAR_MIRROR_DIR        1             // 1=左右齿轮舵机背靠背镜像安装(自动反向同步); 0=同向安装

#define SERVO_STEP_MS          20            // 每 20ms 微步更新一次
#define SERVO_MAX_STEP_DEG     2             // 上半身单步最大角度: 2度/20ms
#define GEAR_SYNC_STEP_DEG     2             // 底部双齿轮同步单步角度: 2度/20ms(限制双舵机同步电流)
#define SERVO_HOLD_TICKS       6             // 到位保持 120ms 后自动切断 PWM 省电
#define SERVO_COOLDOWN_TICKS   3             // 切换间歇 60ms 供板载5V电容回充

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
    {SERVO_PAN_GPIO,        LEDC_CHANNEL_1, 90, 90, 30, 150, 0, false}, // 0: 头部左右
    {SERVO_TILT_GPIO,       LEDC_CHANNEL_2, 90, 90, 55, 125, 0, false}, // 1: 头部上下
    {SERVO_LEFT_ARM_GPIO,   LEDC_CHANNEL_3, 90, 90, 20, 160, 0, false}, // 2: 左手
    {SERVO_RIGHT_ARM_GPIO,  LEDC_CHANNEL_4, 90, 90, 20, 160, 0, false}, // 3: 右手
    {SERVO_LEFT_GEAR_GPIO,  LEDC_CHANNEL_5, 90, 90, 10, 170, 0, false}, // 4: 5号左齿轮履带(同步组)
    {SERVO_RIGHT_GEAR_GPIO, LEDC_CHANNEL_6, 90, 90, 10, 170, 0, false}  // 5: 6号右齿轮履带(同步组)
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

// 底部 5号、6号双齿轮履带【同步直行/后退/转向】封装函数
// delta_deg: 相对90度中心的转动行程 (正数=前进, 负数=后退, 0=归中停止)
// turn_mode: 0=直行同步(自动镜像补偿), 1=原地左转, 2=原地右转
static void SetGearTrackSync(int delta_deg, int turn_mode = 0) {
    int left_target = 90 + delta_deg;
    int right_target = 90 + delta_deg;

    if (turn_mode == 0) {
        // 同步直行前进/后退：若左右舵机背靠背镜像安装，右轮角度自动取反 (90 - delta_deg)
        right_target = (GEAR_MIRROR_DIR == 1) ? (90 - delta_deg) : (90 + delta_deg);
    } else if (turn_mode == 1) {
        // 原地左转：左轮后退，右轮前进
        left_target  = 90 - std::abs(delta_deg);
        right_target = (GEAR_MIRROR_DIR == 1) ? (90 - std::abs(delta_deg)) : (90 + std::abs(delta_deg));
    } else if (turn_mode == 2) {
        // 原地右转：左轮前进，右轮后退
        left_target  = 90 + std::abs(delta_deg);
        right_target = (GEAR_MIRROR_DIR == 1) ? (90 + std::abs(delta_deg)) : (90 - std::abs(delta_deg));
    }

    SetServoTargetSafe(GEAR_LEFT_IDX, left_target);
    SetServoTargetSafe(GEAR_RIGHT_IDX, right_target);
}

static void StaggeredServoTask(void* arg) {
    int active_upper_idx = -1; // 当前激活的上半身舵机 (0~3)
    bool gears_sync_active = false; // 底部 5/6 号双齿轮是否正在同步运转
    int cooldown_ticks = 0;
    int auto_reset_ticks = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(SERVO_STEP_MS));

        // 1. 若当前全部空闲，先检查上半身(0~3)是否有动作，再检查底部双齿轮(4 & 5)是否需要同步动作
        if (active_upper_idx == -1 && !gears_sync_active) {
            if (cooldown_ticks > 0) {
                cooldown_ticks--;
                continue;
            }
            // 优先排队执行上半身 0~3 号舵机 (单路错峰)
            for (int i = 0; i < SERVO_UPPER_NUM; i++) {
                if (g_servos[i].current_angle != g_servos[i].target_angle) {
                    active_upper_idx = i;
                    g_servos[i].hold_counter = 0;
                    break;
                }
            }
            // 当上半身 0~3 号舵机全部静止时，若 5号或6号齿轮有目标行程，立即开启【双齿轮同步模式】
            if (active_upper_idx == -1) {
                if (g_servos[GEAR_LEFT_IDX].current_angle != g_servos[GEAR_LEFT_IDX].target_angle ||
                    g_servos[GEAR_RIGHT_IDX].current_angle != g_servos[GEAR_RIGHT_IDX].target_angle) {
                    gears_sync_active = true;
                    g_servos[GEAR_LEFT_IDX].hold_counter = 0;
                    g_servos[GEAR_RIGHT_IDX].hold_counter = 0;
                }
            }
        }

        // 2A. 执行上半身单舵机错峰移动 (0~3号每次只动1台)
        if (active_upper_idx != -1) {
            EspIdfStaggeredServo &s = g_servos[active_upper_idx];
            if (s.current_angle != s.target_angle) {
                int diff = s.target_angle - s.current_angle;
                s.current_angle += (std::abs(diff) <= SERVO_MAX_STEP_DEG) ? diff : ((diff > 0) ? SERVO_MAX_STEP_DEG : -SERVO_MAX_STEP_DEG);
                ServoWriteAngle(active_upper_idx, s.current_angle);
            } else {
                s.hold_counter++;
                if (s.hold_counter >= SERVO_HOLD_TICKS) {
                    ServoDetach(active_upper_idx);
                    active_upper_idx = -1;
                    cooldown_ticks = SERVO_COOLDOWN_TICKS;
                }
            }
            continue;
        }

        // 2B. 执行底部 5号 & 6号双齿轮【严格同帧同步】移动
        if (gears_sync_active) {
            EspIdfStaggeredServo &sL = g_servos[GEAR_LEFT_IDX];
            EspIdfStaggeredServo &sR = g_servos[GEAR_RIGHT_IDX];
            bool left_moving  = (sL.current_angle != sL.target_angle);
            bool right_moving = (sR.current_angle != sR.target_angle);

            if (left_moving || right_moving) {
                if (left_moving) {
                    int diffL = sL.target_angle - sL.current_angle;
                    sL.current_angle += (std::abs(diffL) <= GEAR_SYNC_STEP_DEG) ? diffL : ((diffL > 0) ? GEAR_SYNC_STEP_DEG : -GEAR_SYNC_STEP_DEG);
                }
                if (right_moving) {
                    int diffR = sR.target_angle - sR.current_angle;
                    sR.current_angle += (std::abs(diffR) <= GEAR_SYNC_STEP_DEG) ? diffR : ((diffR > 0) ? GEAR_SYNC_STEP_DEG : -GEAR_SYNC_STEP_DEG);
                }
                // 在同一毫秒内同时刷新 5号和 6号 PWM 占空比，确保双侧齿轮 100% 同步啮合推进
                ServoWriteAngle(GEAR_LEFT_IDX, sL.current_angle);
                ServoWriteAngle(GEAR_RIGHT_IDX, sR.current_angle);
            } else {
                // 双齿轮均到达目标位置，保持 120ms 稳住履带后同时断电释放
                sL.hold_counter++;
                if (sL.hold_counter >= SERVO_HOLD_TICKS) {
                    ServoDetach(GEAR_LEFT_IDX);
                    ServoDetach(GEAR_RIGHT_IDX);
                    gears_sync_active = false;
                    cooldown_ticks = SERVO_COOLDOWN_TICKS;
                }
            }
            continue;
        }

        // 3. 全部空闲时，检测摸头(GPIO8)与左右红外避障(GPIO17/18)
        if (auto_reset_ticks > 0) {
            auto_reset_ticks--;
            if (auto_reset_ticks == 0) {
                for (int i = 0; i < SERVO_UPPER_NUM; i++) SetServoTargetSafe(i, 90);
                SetGearTrackSync(0, 0); // 底部双齿轮同步归中
            }
        } else {
            if (gpio_get_level(SENSOR_TOUCH_GPIO) == 1) {
                // 摸头互动：先抬头+挥双手(依次错峰) -> 最后底部双齿轮同步向前滚动一步
                SetServoTargetSafe(1, 65);
                SetServoTargetSafe(2, 145);
                SetServoTargetSafe(3, 35);
                SetGearTrackSync(50, 0); // 5/6号齿轮同步前进50度
                auto_reset_ticks = 75;
            } else if (gpio_get_level(SENSOR_IR_LEFT_GPIO) == 0) {
                // 左侧遇障：头向左看 -> 底部双齿轮同步向右转向避障
                SetServoTargetSafe(0, 135);
                SetGearTrackSync(45, 2); // 双齿轮同步右转
                auto_reset_ticks = 75;
            } else if (gpio_get_level(SENSOR_IR_RIGHT_GPIO) == 0) {
                // 右侧遇障：头向右看 -> 底部双齿轮同步向左转向避障
                SetServoTargetSafe(0, 45);
                SetGearTrackSync(45, 1); // 双齿轮同步左转
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
        "控制机器人上半身四舵机错峰转动与底部双齿轮履带(gear_sync)同步驱动。"
        "参数说明: head_lr为头左右(30~150)；head_ud为头上下(55~125)；"
        "left_hand为左手(20~160)；right_hand为右手(20~160)；"
        "gear_sync为底部双齿轮同步直行参数(90=停止, 150=同步前进, 30=同步后退)。",
        PropertyList({
            Property("head_lr", kPropertyTypeInteger, 90, 30, 150),
            Property("head_ud", kPropertyTypeInteger, 90, 55, 125),
            Property("left_hand", kPropertyTypeInteger, 90, 20, 160),
            Property("right_hand", kPropertyTypeInteger, 90, 20, 160),
            Property("gear_sync", kPropertyTypeInteger, 90, 20, 160)
        }),
        [](const PropertyList& properties) -> ReturnValue {
            SetServoTargetSafe(0, properties["head_lr"].value<int>());
            SetServoTargetSafe(1, properties["head_ud"].value<int>());
            SetServoTargetSafe(2, properties["left_hand"].value<int>());
            SetServoTargetSafe(3, properties["right_hand"].value<int>());
            // 将 gear_sync 转换为相对 90 度的同步位移量，驱动 5、6 号双齿轮同步旋转
            int sync_val = properties["gear_sync"].value<int>();
            SetGearTrackSync(sync_val - 90, 0);
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

        // 1. 启动上半身4舵机错峰 + 底部5/6号双齿轮同步控制
        InitStaggeredServosAndSensors();

        // 2. 注册预设动作工具（含底部双齿轮履带同步前进/后退/转向）
        McpServer::GetInstance().AddTool("self.servo.do_action",
            "控制机器人做出连贯肢体与底盘履带同步动作。action_type取值："
            "1=点头, 2=摇头, 3=举左手, 4=举右手, 5=双手欢呼, "
            "6=底盘双齿轮同步前进, 7=底盘双齿轮同步后退, 8=原地左转, 9=原地右转, 0=全部立正归位。",
            PropertyList({
                Property("action_type", kPropertyTypeInteger, 0, 9)
            }),
            [](const PropertyList& properties) -> ReturnValue {
                int action_type = properties["action_type"].value<int>();
                switch (action_type) {
                    case 1: SetServoTargetSafe(1, 60); break;
                    case 2: SetServoTargetSafe(0, 135); break;
                    case 3: SetServoTargetSafe(2, 150); break;
                    case 4: SetServoTargetSafe(3, 30); break;
                    case 5:
                        SetServoTargetSafe(1, 65);
                        SetServoTargetSafe(2, 150);
                        SetServoTargetSafe(3, 30);
                        break;
                    case 6: // 5号、6号双齿轮同步前进
                        SetGearTrackSync(65, 0);
                        break;
                    case 7: // 5号、6号双齿轮同步后退
                        SetGearTrackSync(-65, 0);
                        break;
                    case 8: // 5号、6号双齿轮同步原地左转
                        SetGearTrackSync(55, 1);
                        break;
                    case 9: // 5号、6号双齿轮同步原地右转
                        SetGearTrackSync(55, 2);
                        break;
                    case 0:
                    default:
                        for (int i = 0; i < SERVO_UPPER_NUM; i++) SetServoTargetSafe(i, 90);
                        SetGearTrackSync(0, 0);
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
