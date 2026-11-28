#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <esp_random.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lvgl.h>
#include <atomic>
#include <cmath>
#include <cstring>

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

#define TAG "CompactWifiBoardLCD"

// 如果烧录后发现点阵颜色红蓝错乱或发花，把此项改为 1 重新编译即可
#define MATRIX_SWAP_RGB565     0
// 200x200 居中画布垂直偏移（像素）：负值上移留给底部字幕，正值下移留给顶部状态栏
#define MATRIX_OFFSET_Y        0
// 画布圆角半径（像素）：0 为直角，16 为柔和圆角窗口
#define MATRIX_CORNER_RADIUS   16

// ==================== 100% 原创 200x200 (40x40灯珠) 居中发光点阵表情引擎 ====================
namespace matrix_face {

enum Emotion : int {
    kNeutral = 0,
    kHappy,
    kSurprised,
    kThinking,
    kCool,
    kSad,
    kAngry,
    kSleepy,
    kLoving,
    kWinking
};

// 8色发光调色板: 0青蓝, 1暖白, 2樱花粉, 3薄荷绿, 4琥珀金, 5珊瑚红, 6柔紫, 7冰天蓝
constexpr uint8_t kPalette[8][3] = {
    { 65, 215, 255},
    {255, 240, 215},
    {255, 105, 165},
    { 90, 240, 145},
    {255, 195,  55},
    {255,  75,  65},
    {185, 125, 255},
    {135, 235, 255}
};

class DotMatrixEngine {
public:
    static constexpr int kCols = 40;
    static constexpr int kRows = 40;
    static constexpr int kCell = 5;
    static constexpr int kWidth  = kCols * kCell;  // 200px
    static constexpr int kHeight = kRows * kCell;  // 200px
    static constexpr int kCells  = kCols * kRows;  // 1600颗灯珠

    ~DotMatrixEngine() {
        if (fb_) free(fb_);
        if (bloom_) free(bloom_);
    }

    bool Init() {
        // 200x200 帧缓冲 (78KB) 放 PSRAM，节省内部 DRAM
        fb_ = static_cast<uint16_t*>(
            heap_caps_malloc(kWidth * kHeight * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!fb_) {
            fb_ = static_cast<uint16_t*>(malloc(kWidth * kHeight * sizeof(uint16_t)));
        }
        // 光晕缓冲 (4.7KB) 优先分配内部 SRAM，减少 PSRAM 总线争抢
        bloom_ = static_cast<uint8_t*>(
            heap_caps_malloc(kCells * 3, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        if (!bloom_) {
            bloom_ = static_cast<uint8_t*>(
                heap_caps_malloc(kCells * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        }
        if (!bloom_) {
            bloom_ = static_cast<uint8_t*>(malloc(kCells * 3));
        }
        if (!fb_ || !bloom_) {
            ESP_LOGE(TAG, "DotMatrixEngine buffer alloc failed");
            return false;
        }

        PrecomputeMasks();
        memset(grid_, 0, sizeof(grid_));
        memset(fb_, 0, kWidth * kHeight * sizeof(uint16_t));
        next_blink_t_ = 2.6f;
        next_saccade_t_ = 1.8f;
        return true;
    }

    uint16_t* Framebuffer() const { return fb_; }

    void StepAndRender(float dt, Emotion emotion, DeviceState state, float gaze_hint_x, int theme_color_idx) {
        if (!fb_ || !bloom_) return;
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.12f) dt = 0.12f;
        time_s_ += dt;

        UpdateMotion(dt, emotion, state, gaze_hint_x);
        memset(grid_, 0, sizeof(grid_));

        uint8_t eye_color = PickEyeColor(emotion, state, theme_color_idx);
        DrawExpression(emotion, state, eye_color);
        DrawStateOverlay(emotion, state, eye_color);
        RenderGlowFrame();
    }

private:
    static inline float ClampF(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    static inline uint16_t PackRgb565(uint8_t r, uint8_t g, uint8_t b) {
        uint16_t c = static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
#if MATRIX_SWAP_RGB565
        c = static_cast<uint16_t>((c >> 8) | (c << 8));
#endif
        return c;
    }

    static float Rand01() {
        return static_cast<float>(esp_random() & 0xFFFF) / 65535.0f;
    }

    void PrecomputeMasks() {
        for (int val = 0; val < 256; ++val) {
            int lvl = val & 31;
            int cid = val >> 5;
            float b = static_cast<float>(lvl) / 31.0f;
            color_lut_[val][0] = static_cast<uint8_t>(kPalette[cid][0] * b + 0.5f);
            color_lut_[val][1] = static_cast<uint8_t>(kPalette[cid][1] * b + 0.5f);
            color_lut_[val][2] = static_cast<uint8_t>(kPalette[cid][2] * b + 0.5f);
            bright_lut_[val]   = static_cast<uint8_t>(b * 255.0f + 0.5f);
            hot_lut_[val]      = static_cast<uint16_t>(b * b * 0.42f * 256.0f + 0.5f);
        }

        const float center = (kCell - 1) * 0.5f;  // 2.0
        for (int sy = 0; sy < kCell; ++sy) {
            for (int sx = 0; sx < kCell; ++sx) {
                float dx = sx - center;
                float dy = sy - center;
                float dist = sqrtf(dx * dx + dy * dy);

                // 圆形灯珠边缘平滑衰减 + 中心高光 + 未亮灯珠暗灰底纹
                float dot_f   = ClampF((2.25f - dist) / 0.85f, 0.0f, 1.0f);
                float core_f  = ClampF((1.15f - dist) / 0.80f, 0.0f, 1.0f);
                float unlit_f = ClampF((1.75f - dist) / 0.90f, 0.0f, 1.0f);
                float halo_f  = 0.82f * (1.0f - 0.55f * dot_f);

                dot_w_[sy][sx]   = static_cast<uint16_t>(dot_f * 256.0f + 0.5f);
                core_w_[sy][sx]  = static_cast<uint16_t>(core_f * 256.0f + 0.5f);
                unlit_w_[sy][sx] = static_cast<uint16_t>(unlit_f * 256.0f + 0.5f);
                halo_w_[sy][sx]  = static_cast<uint16_t>(halo_f * 256.0f + 0.5f);

                uint8_t ur = static_cast<uint8_t>(4 + 16 * unlit_f);
                uint8_t ug = static_cast<uint8_t>(5 + 17 * unlit_f);
                uint8_t ub = static_cast<uint8_t>(8 + 20 * unlit_f);
                unlit_tile_[sy][sx] = PackRgb565(ur, ug, ub);
            }
        }
    }

    void PlotCell(int x, int y, uint8_t color_idx, float intensity) {
        if (x < 0 || x >= kCols || y < 0 || y >= kRows) return;
        if (intensity <= 0.01f) return;
        int lvl = static_cast<int>(ClampF(intensity, 0.0f, 1.0f) * 31.0f + 0.5f);
        if (lvl <= 0) return;
        if (lvl > 31) lvl = 31;
        uint8_t& dst = grid_[y * kCols + x];
        if (lvl >= (dst & 31)) {
            dst = static_cast<uint8_t>(((color_idx & 7) << 5) | lvl);
        }
    }

    static float SdRoundBox(float px, float py, float hw, float hh, float r) {
        float qx = fabsf(px) - (hw - r);
        float qy = fabsf(py) - (hh - r);
        float ox = qx > 0.0f ? qx : 0.0f;
        float oy = qy > 0.0f ? qy : 0.0f;
        float inside = (qx > qy ? qx : qy);
        if (inside > 0.0f) inside = 0.0f;
        return sqrtf(ox * ox + oy * oy) + inside - r;
    }

    void DrawStyledEye(float cx, float cy, float hw, float hh, float radius,
                       float lid_cut_y, float lid_slope, uint8_t color_idx, float gain) {
        // 半径限幅：防止眨眼高度变窄时眼睛横向缩成小圆点
        float r = fminf(radius, fminf(hw, hh));
        if (r < 0.0f) r = 0.0f;

        int x0 = static_cast<int>(floorf(cx - hw - 2.0f));
        int x1 = static_cast<int>(ceilf(cx + hw + 2.0f));
        int y0 = static_cast<int>(floorf(cy - hh - 2.0f));
        int y1 = static_cast<int>(ceilf(cy + hh + 2.0f));

        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                float px = static_cast<float>(x) - cx;
                float py = static_cast<float>(y) - cy;
                float d = SdRoundBox(px, py, hw, hh, r);
                if (lid_cut_y > -20.0f) {
                    float d_lid = (lid_cut_y + lid_slope * px) - py;
                    if (d_lid > d) d = d_lid;
                }
                float cov = ClampF(0.85f - d, 0.0f, 1.0f);
                if (cov > 0.02f) {
                    float shade = 0.88f + 0.12f * ClampF(-py / (hh + 1.0f), 0.0f, 1.0f);
                    PlotCell(x, y, color_idx, cov * gain * shade);
                }
            }
        }
    }

    // 弯月笑眼 (^ ^)
    void DrawHappyArcEye(float cx, float cy, float span_x, float arch_h, float thickness, uint8_t color_idx, float gain) {
        int x0 = static_cast<int>(floorf(cx - span_x - 2.0f));
        int x1 = static_cast<int>(ceilf(cx + span_x + 2.0f));
        int y0 = static_cast<int>(floorf(cy - arch_h - 3.0f));
        int y1 = static_cast<int>(ceilf(cy + 4.0f));
        float inv_span = 1.0f / span_x;

        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                float dx = static_cast<float>(x) - cx;
                float nx = ClampF(dx * inv_span, -1.0f, 1.0f);
                float curve_x = cx + nx * span_x;
                float curve_y = cy - arch_h * (1.0f - nx * nx);
                float dist = sqrtf((x - curve_x) * (x - curve_x) + (y - curve_y) * (y - curve_y));
                float cov = ClampF(thickness + 0.55f - dist, 0.0f, 1.0f);
                if (cov > 0.02f) {
                    PlotCell(x, y, color_idx, cov * gain);
                }
            }
        }
    }

    // 惊讶圆环眼 (O O)
    void DrawRingEye(float cx, float cy, float outer_r, float ring_thick, uint8_t color_idx, float gain) {
        int r_lim = static_cast<int>(ceilf(outer_r + 2.0f));
        int icx = static_cast<int>(cx + 0.5f);
        int icy = static_cast<int>(cy + 0.5f);
        for (int y = icy - r_lim; y <= icy + r_lim; ++y) {
            for (int x = icx - r_lim; x <= icx + r_lim; ++x) {
                float dx = x - cx;
                float dy = y - cy;
                float dist = sqrtf(dx * dx + dy * dy);
                float d_ring = fabsf(dist - (outer_r - ring_thick * 0.5f)) - ring_thick * 0.5f;
                float cov = ClampF(0.75f - d_ring, 0.0f, 1.0f);
                float d_pupil = dist - 1.3f;
                float cov_p = ClampF(0.75f - d_pupil, 0.0f, 1.0f) * 0.85f;
                float m = cov > cov_p ? cov : cov_p;
                if (m > 0.02f) {
                    PlotCell(x, y, color_idx, m * gain);
                }
            }
        }
    }

    // 椭圆发光腮红 / 泪滴 / 小圆点
    void DrawSoftOval(float cx, float cy, float rx, float ry, uint8_t color_idx, float gain) {
        int x0 = static_cast<int>(floorf(cx - rx - 1.5f));
        int x1 = static_cast<int>(ceilf(cx + rx + 1.5f));
        int y0 = static_cast<int>(floorf(cy - ry - 1.5f));
        int y1 = static_cast<int>(ceilf(cy + ry + 1.5f));
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                float nx = (x - cx) / rx;
                float ny = (y - cy) / ry;
                float d = sqrtf(nx * nx + ny * ny);
                float cov = ClampF((1.15f - d) * 1.6f, 0.0f, 1.0f);
                if (cov > 0.02f) {
                    PlotCell(x, y, color_idx, cov * gain);
                }
            }
        }
    }

    void UpdateMotion(float dt, Emotion emotion, DeviceState state, float gaze_hint_x) {
        bool can_blink = (emotion == kNeutral || emotion == kThinking || emotion == kCool ||
                          emotion == kSad || emotion == kAngry || emotion == kLoving);
        if (can_blink) {
            if (blink_phase_ > 0.0f) {
                blink_phase_ += dt * 7.2f;  // 单次眨眼约 0.28 秒
                if (blink_phase_ >= 2.0f) {
                    blink_phase_ = 0.0f;
                    next_blink_t_ = time_s_ + 2.4f + Rand01() * 3.4f;
                }
            } else if (time_s_ >= next_blink_t_) {
                blink_phase_ = 0.01f;
            }
        } else {
            blink_phase_ = 0.0f;
        }

        if (fabsf(gaze_hint_x) > 0.1f) {
            target_gaze_x_ = ClampF(gaze_hint_x, -3.5f, 3.5f);
            target_gaze_y_ = -0.5f;
        } else if (state == kDeviceStateListening) {
            target_gaze_x_ = 0.0f;
            target_gaze_y_ = -0.6f;
        } else if (emotion == kThinking) {
            target_gaze_x_ = -2.8f;
            target_gaze_y_ = -1.3f;
        } else if (emotion == kCool) {
            target_gaze_x_ = 2.6f;
            target_gaze_y_ = 0.0f;
        } else if (emotion == kSad) {
            target_gaze_x_ = 0.0f;
            target_gaze_y_ = 2.0f;
        } else if (time_s_ >= next_saccade_t_) {
            float r = Rand01();
            if (r < 0.45f) {
                target_gaze_x_ = 0.0f;
                target_gaze_y_ = 0.0f;
            } else {
                target_gaze_x_ = (Rand01() - 0.5f) * 4.8f;
                target_gaze_y_ = (Rand01() - 0.5f) * 2.0f;
            }
            next_saccade_t_ = time_s_ + 1.6f + Rand01() * 2.6f;
        }

        float smooth = ClampF(dt * 9.0f, 0.0f, 1.0f);
        gaze_x_ += (target_gaze_x_ - gaze_x_) * smooth;
        gaze_y_ += (target_gaze_y_ - gaze_y_) * smooth;
    }

    uint8_t PickEyeColor(Emotion emotion, DeviceState state, int theme_color_idx) const {
        if (emotion == kAngry)     return 5;  // 珊瑚红
        if (emotion == kLoving)    return 2;  // 樱花粉
        if (emotion == kSurprised) return 4;  // 琥珀金
        if (emotion == kThinking)  return 6;  // 柔紫
        if (emotion == kSad)       return 7;  // 冰天蓝
        if (emotion == kHappy)     return 1;  // 暖白（配樱花粉腮红）
        if (state == kDeviceStateListening) return 3;  // 薄荷绿
        if (state == kDeviceStateSpeaking)  return 0;  // 亮青蓝
        return static_cast<uint8_t>(theme_color_idx & 7);
    }

    void DrawExpression(Emotion emotion, DeviceState state, uint8_t eye_color) {
        float breath = 0.90f + 0.10f * sinf(time_s_ * 2.3f);
        float open_ratio = 1.0f;
        if (blink_phase_ > 0.0f) {
            float t = blink_phase_ <= 1.0f ? blink_phase_ : (2.0f - blink_phase_);
            open_ratio = 1.0f - 0.92f * t;
        }

        float shake_x = 0.0f;
        float bounce_y = 0.0f;
        if (emotion == kSurprised || emotion == kAngry) {
            shake_x = sinf(time_s_ * 34.0f) * 0.6f;
        }
        if (state == kDeviceStateSpeaking) {
            bounce_y = -fabsf(sinf(time_s_ * 7.5f)) * 0.9f;
        } else if (emotion == kHappy) {
            bounce_y = -fabsf(sinf(time_s_ * 5.0f)) * 1.0f;
        }

        // 40x40 网格对称中心 X = 19.5
        const float lx = 11.5f + gaze_x_ + shake_x;
        const float rx = 27.5f + gaze_x_ + shake_x;
        const float cy = 16.5f + gaze_y_ + bounce_y;

        switch (emotion) {
            case kHappy:
                DrawHappyArcEye(lx, cy + 1.0f, 4.5f, 3.6f, 1.15f, eye_color, 1.0f);
                DrawHappyArcEye(rx, cy + 1.0f, 4.5f, 3.6f, 1.15f, eye_color, 1.0f);
                DrawSoftOval(6.2f  + gaze_x_ * 0.35f, 23.5f + bounce_y * 0.5f, 2.5f, 1.4f, 2, 0.78f);
                DrawSoftOval(32.8f + gaze_x_ * 0.35f, 23.5f + bounce_y * 0.5f, 2.5f, 1.4f, 2, 0.78f);
                break;

            case kLoving:
                DrawHappyArcEye(lx, cy + 0.7f, 4.6f, 4.0f, 1.25f, 2, 1.0f);
                DrawHappyArcEye(rx, cy + 0.7f, 4.6f, 4.0f, 1.25f, 2, 1.0f);
                DrawSoftOval(6.8f,  23.8f, 2.7f, 1.6f, 2, 0.85f * breath);
                DrawSoftOval(32.2f, 23.8f, 2.7f, 1.6f, 2, 0.85f * breath);
                break;

            case kWinking:
                DrawStyledEye(lx, cy, 4.6f, 6.2f * open_ratio, 2.3f, -30.0f, 0.0f, eye_color, breath);
                DrawHappyArcEye(rx, cy + 0.8f, 4.5f, 3.3f, 1.15f, eye_color, 1.0f);
                DrawSoftOval(32.5f, 23.5f, 2.2f, 1.3f, 2, 0.7f);
                break;

            case kSurprised:
                DrawRingEye(lx, cy - 0.4f, 5.2f, 1.9f, eye_color, 1.0f);
                DrawRingEye(rx, cy - 0.4f, 5.2f, 1.9f, eye_color, 1.0f);
                break;

            case kThinking:
                DrawStyledEye(lx, cy - 0.6f, 4.6f, 6.0f * open_ratio, 2.3f, -30.0f, 0.0f, eye_color, breath);
                DrawStyledEye(rx, cy + 0.8f, 4.1f, 3.8f * open_ratio, 1.8f, -30.0f, 0.0f, eye_color, breath);
                break;

            case kCool:
                DrawStyledEye(lx, cy + 0.4f, 5.2f, 4.2f * open_ratio, 1.7f, -1.8f,  0.16f, eye_color, 1.0f);
                DrawStyledEye(rx, cy + 0.4f, 5.2f, 4.2f * open_ratio, 1.7f, -1.8f, -0.16f, eye_color, 1.0f);
                break;

            case kAngry:
                DrawStyledEye(lx, cy, 4.8f, 5.4f * open_ratio, 1.8f, -1.5f,  0.52f, eye_color, 1.0f);
                DrawStyledEye(rx, cy, 4.8f, 5.4f * open_ratio, 1.8f, -1.5f, -0.52f, eye_color, 1.0f);
                break;

            case kSad: {
                DrawStyledEye(lx, cy + 0.8f, 4.6f, 5.1f * open_ratio, 2.0f, -1.3f, -0.45f, eye_color, 0.9f);
                DrawStyledEye(rx, cy + 0.8f, 4.6f, 5.1f * open_ratio, 2.0f, -1.3f,  0.45f, eye_color, 0.9f);
                float tear_y = 22.8f + fmodf(time_s_ * 4.2f, 6.0f);
                DrawSoftOval(lx - 1.6f, tear_y, 1.0f, 1.3f, 0, 0.85f);
                break;
            }

            case kSleepy:
                DrawStyledEye(lx, cy + 1.8f, 4.8f, 1.4f, 0.9f, -30.0f, 0.0f, 6, 0.55f + 0.25f * sinf(time_s_ * 1.4f));
                DrawStyledEye(rx, cy + 1.8f, 4.8f, 1.4f, 0.9f, -30.0f, 0.0f, 6, 0.55f + 0.25f * sinf(time_s_ * 1.4f));
                break;

            case kNeutral:
            default: {
                float hh = (state == kDeviceStateListening) ? 6.8f : 6.0f;
                float hw = (state == kDeviceStateListening) ? 4.9f : 4.5f;
                DrawStyledEye(lx, cy, hw, hh * open_ratio, 2.3f, -30.0f, 0.0f, eye_color, breath);
                DrawStyledEye(rx, cy, hw, hh * open_ratio, 2.3f, -30.0f, 0.0f, eye_color, breath);
                break;
            }
        }
    }

    // 底部嘴型与语音/连接状态 LED 动效（第 28~36 行）
    void DrawStateOverlay(Emotion emotion, DeviceState state, uint8_t eye_color) {
        if (state == kDeviceStateSpeaking) {
            for (int i = -4; i <= 4; ++i) {
                float env = 1.0f - fabsf(static_cast<float>(i)) * 0.16f;
                float wave = 0.5f + 0.5f * sinf(time_s_ * 14.0f + i * 0.95f) * cosf(time_s_ * 5.5f - i * 0.4f);
                int half_h = static_cast<int>(0.8f + env * wave * 2.8f);
                int x = 20 + i * 2;
                for (int dy = -half_h; dy <= half_h; ++dy) {
                    PlotCell(x, 32 + dy, eye_color, 0.95f);
                }
            }
        } else if (state == kDeviceStateListening) {
            for (int i = -3; i <= 3; ++i) {
                float wave = 0.35f + 0.65f * (0.5f + 0.5f * sinf(time_s_ * 8.0f + i * 0.85f));
                int h = static_cast<int>(1.0f + wave * 2.6f);
                int x = 20 + i * 3;
                for (int k = 0; k < h; ++k) {
                    PlotCell(x, 35 - k, 3, 0.92f);
                }
            }
        } else if (state == kDeviceStateConnecting) {
            int active_dot = static_cast<int>(time_s_ * 5.5f) % 5;
            for (int i = 0; i < 5; ++i) {
                float lvl = (i == active_dot) ? 1.0f : 0.25f;
                DrawSoftOval(11.5f + i * 4.0f, 32.0f, 1.1f, 1.1f, 0, lvl);
            }
        } else if (emotion == kHappy || emotion == kLoving) {
            for (int x = 16; x <= 23; ++x) {
                float nx = (static_cast<float>(x) - 19.5f) / 3.5f;
                float y = 30.2f - 1.5f * nx * nx;
                PlotCell(x, static_cast<int>(y + 0.5f), 2, 0.92f);
            }
        } else if (emotion == kSurprised) {
            DrawRingEye(19.5f, 30.5f, 2.3f, 1.0f, 4, 0.9f);
        }
    }

    // 5行循环 SRAM 缓冲实现 [1 4 6 4 1]/16 可分离泛光模糊 + 活跃格剔除高速渲染至 200x200 RGB565
    void RenderGlowFrame() {
        static constexpr uint8_t kK[5] = {1, 4, 6, 4, 1};

        for (int r = 0; r < kRows + 2; ++r) {
            if (r < kRows) {
                const uint8_t* row_in = grid_ + r * kCols;
                uint16_t* row_dst = row_ring_[r % 5];
                for (int c = 0; c < kCols; ++c) {
                    uint16_t sr = 0, sg = 0, sb = 0;
                    for (int k = -2; k <= 2; ++k) {
                        int cc = c + k;
                        if (cc < 0 || cc >= kCols) continue;
                        uint8_t v = row_in[cc];
                        if (v == 0) continue;
                        uint8_t w = kK[k + 2];
                        sr += w * color_lut_[v][0];
                        sg += w * color_lut_[v][1];
                        sb += w * color_lut_[v][2];
                    }
                    row_dst[c * 3 + 0] = sr;
                    row_dst[c * 3 + 1] = sg;
                    row_dst[c * 3 + 2] = sb;
                }
            }
            const int out_r = r - 2;
            if (out_r < 0) continue;
            uint8_t* row_out = bloom_ + out_r * kCols * 3;
            for (int c = 0; c < kCols; ++c) {
                uint32_t sr = 0, sg = 0, sb = 0;
                for (int k = -2; k <= 2; ++k) {
                    int rr = out_r + k;
                    if (rr < 0 || rr >= kRows) continue;
                    const uint16_t* src = row_ring_[rr % 5] + c * 3;
                    uint8_t w = kK[k + 2];
                    sr += w * src[0];
                    sg += w * src[1];
                    sb += w * src[2];
                }
                row_out[c * 3 + 0] = static_cast<uint8_t>(sr >> 8);
                row_out[c * 3 + 1] = static_cast<uint8_t>(sg >> 8);
                row_out[c * 3 + 2] = static_cast<uint8_t>(sb >> 8);
            }
        }

        // 逐灯珠合成 5x5 像素块写入 200x200 帧缓冲
        for (int r = 0; r < kRows; ++r) {
            const int base_y = r * kCell;
            for (int c = 0; c < kCols; ++c) {
                const int idx = r * kCols + c;
                const uint8_t val = grid_[idx];
                const uint8_t br = bloom_[idx * 3 + 0];
                const uint8_t bg = bloom_[idx * 3 + 1];
                const uint8_t bb = bloom_[idx * 3 + 2];
                const int base_x = c * kCell;

                if (val == 0 && (br | bg | bb) < 3) {
                    for (int sy = 0; sy < kCell; ++sy) {
                        uint16_t* dst = fb_ + (base_y + sy) * kWidth + base_x;
                        memcpy(dst, unlit_tile_[sy], kCell * sizeof(uint16_t));
                    }
                    continue;
                }

                const uint8_t lr = color_lut_[val][0];
                const uint8_t lg = color_lut_[val][1];
                const uint8_t lb = color_lut_[val][2];
                const uint16_t hot_gain = hot_lut_[val];
                const uint16_t dark_gain = 255 - bright_lut_[val];

                for (int sy = 0; sy < kCell; ++sy) {
                    uint16_t* dst = fb_ + (base_y + sy) * kWidth + base_x;
                    for (int sx = 0; sx < kCell; ++sx) {
                        const uint16_t dw = dot_w_[sy][sx];
                        const uint16_t hw = halo_w_[sy][sx];
                        const uint16_t cw = static_cast<uint16_t>((core_w_[sy][sx] * hot_gain) >> 8);
                        const uint16_t uw = static_cast<uint16_t>((unlit_w_[sy][sx] * dark_gain) >> 8);

                        uint32_t pr = ((lr * dw) >> 8) + ((255 * cw) >> 8) + ((br * hw) >> 8) + ((18 * uw) >> 8) + 4;
                        uint32_t pg = ((lg * dw) >> 8) + ((250 * cw) >> 8) + ((bg * hw) >> 8) + ((20 * uw) >> 8) + 5;
                        uint32_t pb = ((lb * dw) >> 8) + ((240 * cw) >> 8) + ((bb * hw) >> 8) + ((24 * uw) >> 8) + 7;

                        if (pr > 255) pr = 255;
                        if (pg > 255) pg = 255;
                        if (pb > 255) pb = 255;
                        dst[sx] = PackRgb565(static_cast<uint8_t>(pr), static_cast<uint8_t>(pg), static_cast<uint8_t>(pb));
                    }
                }
            }
        }
    }

    uint16_t* fb_ = nullptr;
    uint8_t*  bloom_ = nullptr;
    uint8_t   grid_[kCells];
    uint16_t  row_ring_[5][kCols * 3];  // 1.2KB 内部 SRAM 循环行缓存

    uint8_t  color_lut_[256][3];
    uint8_t  bright_lut_[256];
    uint16_t hot_lut_[256];
    uint16_t dot_w_[kCell][kCell];
    uint16_t core_w_[kCell][kCell];
    uint16_t unlit_w_[kCell][kCell];
    uint16_t halo_w_[kCell][kCell];
    uint16_t unlit_tile_[kCell][kCell];

    float time_s_ = 0.0f;
    float blink_phase_ = 0.0f;
    float next_blink_t_ = 2.5f;
    float next_saccade_t_ = 2.0f;
    float gaze_x_ = 0.0f;
    float gaze_y_ = 0.0f;
    float target_gaze_x_ = 0.0f;
    float target_gaze_y_ = 0.0f;
};

}  // namespace matrix_face

// 继承 SpiLcdDisplay，居中挂载 200x200 发光点阵画布，保留顶部状态栏与底部字幕
class GlowMatrixLcdDisplay : public SpiLcdDisplay {
public:
    GlowMatrixLcdDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                         int width, int height, int offset_x, int offset_y,
                         bool mirror_x, bool mirror_y, bool swap_xy)
        : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
        if (!engine_.Init()) {
            return;
        }
#if LVGL_VERSION_MAJOR >= 9
        memset(&img_dsc_, 0, sizeof(img_dsc_));
        img_dsc_.header.magic  = LV_IMAGE_HEADER_MAGIC;
        img_dsc_.header.cf     = LV_COLOR_FORMAT_RGB565;
        img_dsc_.header.flags  = 0;
        img_dsc_.header.w      = matrix_face::DotMatrixEngine::kWidth;
        img_dsc_.header.h      = matrix_face::DotMatrixEngine::kHeight;
        img_dsc_.header.stride = matrix_face::DotMatrixEngine::kWidth * sizeof(uint16_t);
        img_dsc_.data_size     = matrix_face::DotMatrixEngine::kWidth *
                                 matrix_face::DotMatrixEngine::kHeight * sizeof(uint16_t);
        img_dsc_.data          = reinterpret_cast<const uint8_t*>(engine_.Framebuffer());

        DisplayLockGuard lock(this);
        face_img_ = lv_image_create(lv_screen_active());
        lv_image_set_src(face_img_, &img_dsc_);
        lv_obj_set_size(face_img_, matrix_face::DotMatrixEngine::kWidth,
                        matrix_face::DotMatrixEngine::kHeight);
        lv_obj_align(face_img_, LV_ALIGN_CENTER, 0, MATRIX_OFFSET_Y);
        lv_obj_set_style_radius(face_img_, MATRIX_CORNER_RADIUS, 0);
        lv_obj_set_style_clip_corner(face_img_, true, 0);
        lv_obj_add_flag(face_img_, LV_OBJ_FLAG_HIDDEN);  // 开机配网阶段默认先隐藏
        last_tick_us_ = esp_timer_get_time();
        anim_timer_ = lv_timer_create(OnAnimTimer, 66, this);  // ~15 FPS，省总线保语音流畅
#endif
    }

    ~GlowMatrixLcdDisplay() override {
        if (anim_timer_) {
            DisplayLockGuard lock(this);
            lv_timer_delete(anim_timer_);
            anim_timer_ = nullptr;
        }
    }

    bool SetEmotion(const char* emotion) override {
        bool ok = SpiLcdDisplay::SetEmotion(emotion);
        if (!emotion) return ok;

        matrix_face::Emotion mapped = matrix_face::kNeutral;
        if (strcmp(emotion, "happy") == 0 || strcmp(emotion, "laughing") == 0 || strcmp(emotion, "funny") == 0) {
            mapped = matrix_face::kHappy;
        } else if (strcmp(emotion, "loving") == 0 || strcmp(emotion, "kissy") == 0) {
            mapped = matrix_face::kLoving;
        } else if (strcmp(emotion, "winking") == 0) {
            mapped = matrix_face::kWinking;
        } else if (strcmp(emotion, "surprised") == 0 || strcmp(emotion, "shocked") == 0) {
            mapped = matrix_face::kSurprised;
        } else if (strcmp(emotion, "thinking") == 0 || strcmp(emotion, "confused") == 0) {
            mapped = matrix_face::kThinking;
        } else if (strcmp(emotion, "cool") == 0 || strcmp(emotion, "confident") == 0) {
            mapped = matrix_face::kCool;
        } else if (strcmp(emotion, "sad") == 0 || strcmp(emotion, "crying") == 0) {
            mapped = matrix_face::kSad;
        } else if (strcmp(emotion, "angry") == 0 || strcmp(emotion, "embarrassed") == 0) {
            mapped = matrix_face::kAngry;
        } else if (strcmp(emotion, "sleepy") == 0) {
            mapped = matrix_face::kSleepy;
        }
        current_emotion_.store(static_cast<int>(mapped));
        return ok;
    }

    void SetGazeHint(float gaze_x) {
        gaze_hint_milli_.store(static_cast<int>(gaze_x * 1000.0f));
    }

    void SetMatrixEnabled(bool enabled) {
        enabled_.store(enabled);
    }

    void SetThemeColor(int color_idx) {
        theme_color_.store(color_idx & 7);
    }

private:
    static void OnAnimTimer(lv_timer_t* timer) {
        auto* self = static_cast<GlowMatrixLcdDisplay*>(lv_timer_get_user_data(timer));
        if (self) self->UpdateFrame();
    }

    void UpdateFrame() {
#if LVGL_VERSION_MAJOR >= 9
        if (!face_img_) return;

        DeviceState state = Application::GetInstance().GetDeviceState();
        // 仅在待机、聆听、说话、连接态显示点阵表情；开机/配网/激活码/OTA 升级自动隐藏不遮挡
        bool should_show = enabled_.load() && (
            state == kDeviceStateIdle ||
            state == kDeviceStateListening ||
            state == kDeviceStateSpeaking ||
            state == kDeviceStateConnecting
        );

        if (!should_show) {
            if (is_visible_) {
                lv_obj_add_flag(face_img_, LV_OBJ_FLAG_HIDDEN);
                is_visible_ = false;
            }
            last_tick_us_ = esp_timer_get_time();
            return;
        }

        if (!is_visible_) {
            lv_obj_remove_flag(face_img_, LV_OBJ_FLAG_HIDDEN);
            is_visible_ = true;
        }

        int64_t now_us = esp_timer_get_time();
        float dt = static_cast<float>(now_us - last_tick_us_) * 1e-6f;
        last_tick_us_ = now_us;

        auto emo = static_cast<matrix_face::Emotion>(current_emotion_.load());
        float gaze_hint = static_cast<float>(gaze_hint_milli_.load()) * 0.001f;
        engine_.StepAndRender(dt, emo, state, gaze_hint, theme_color_.load());
        lv_obj_invalidate(face_img_);
#endif
    }

    matrix_face::DotMatrixEngine engine_;
#if LVGL_VERSION_MAJOR >= 9
    lv_image_dsc_t img_dsc_;
#endif
    lv_obj_t*   face_img_ = nullptr;
    lv_timer_t* anim_timer_ = nullptr;
    int64_t     last_tick_us_ = 0;
    bool        is_visible_ = false;

    std::atomic<int>  current_emotion_{static_cast<int>(matrix_face::kNeutral)};
    std::atomic<int>  gaze_hint_milli_{0};
    std::atomic<bool> enabled_{true};
    std::atomic<int>  theme_color_{0};
};

static GlowMatrixLcdDisplay* g_matrix_display = nullptr;

// ==================== 4舵机错峰 + 5/6号双齿轮履带【同步驱动】+ 头部三路传感器模块 ====================
// 6台舵机引脚（对应扩展板背面 14 / 13 / 12 / 11 / 10 / 9 焊点）
#define SERVO_PAN_GPIO         GPIO_NUM_14   // 1号舵机: 头部左右摇头 (head_lr)
#define SERVO_TILT_GPIO        GPIO_NUM_13   // 2号舵机: 头部上下点头 (head_ud)
#define SERVO_LEFT_ARM_GPIO    GPIO_NUM_12   // 3号舵机: 左手挥臂 (left_hand)
#define SERVO_RIGHT_ARM_GPIO   GPIO_NUM_11   // 4号舵机: 右手挥臂 (right_hand)
#define SERVO_LEFT_GEAR_GPIO   GPIO_NUM_10   // 5号舵机: 左侧齿轮履带 (left_gear)
#define SERVO_RIGHT_GEAR_GPIO  GPIO_NUM_9    // 6号舵机: 右侧齿轮履带 (right_gear)

// 头部三路传感器引脚（对应扩展板背面 8 / 17 / 18 焊点，零冲突）：
#define SENSOR_TOUCH_GPIO      GPIO_NUM_8    // 头顶 TTP223 触摸模块 (焊在背面 8 号脚, 高电平触发)
#define SENSOR_IR_LEFT_GPIO    GPIO_NUM_17   // 头部左侧红外避障模块 (焊在背面 17 号脚, 低电平触发)
#define SENSOR_IR_RIGHT_GPIO   GPIO_NUM_18   // 头部右侧红外避障模块 (焊在背面 18 号脚, 低电平触发)

#define SERVO_UPPER_NUM        4
#define SERVO_TOTAL_NUM        6
#define GEAR_LEFT_IDX          4
#define GEAR_RIGHT_IDX         5
#define GEAR_MIRROR_DIR        1

#define SERVO_STEP_MS          20
#define SERVO_MAX_STEP_DEG     2
#define GEAR_SYNC_STEP_DEG     2
#define SERVO_HOLD_TICKS       6
#define SERVO_COOLDOWN_TICKS   3

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
    {SERVO_PAN_GPIO,        LEDC_CHANNEL_1, 90, 90, 30, 150, 0, false},
    {SERVO_TILT_GPIO,       LEDC_CHANNEL_2, 90, 90, 55, 125, 0, false},
    {SERVO_LEFT_ARM_GPIO,   LEDC_CHANNEL_3, 90, 90, 20, 160, 0, false},
    {SERVO_RIGHT_ARM_GPIO,  LEDC_CHANNEL_4, 90, 90, 20, 160, 0, false},
    {SERVO_LEFT_GEAR_GPIO,  LEDC_CHANNEL_5, 90, 90, 10, 170, 0, false},
    {SERVO_RIGHT_GEAR_GPIO, LEDC_CHANNEL_6, 90, 90, 10, 170, 0, false}
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

static void SetGearTrackSync(int delta_deg, int turn_mode = 0) {
    int left_target = 90 + delta_deg;
    int right_target = 90 + delta_deg;

    if (turn_mode == 0) {
        right_target = (GEAR_MIRROR_DIR == 1) ? (90 - delta_deg) : (90 + delta_deg);
    } else if (turn_mode == 1) {
        left_target  = 90 - std::abs(delta_deg);
        right_target = (GEAR_MIRROR_DIR == 1) ? (90 - std::abs(delta_deg)) : (90 + std::abs(delta_deg));
    } else if (turn_mode == 2) {
        left_target  = 90 + std::abs(delta_deg);
        right_target = (GEAR_MIRROR_DIR == 1) ? (90 + std::abs(delta_deg)) : (90 - std::abs(delta_deg));
    }

    SetServoTargetSafe(GEAR_LEFT_IDX, left_target);
    SetServoTargetSafe(GEAR_RIGHT_IDX, right_target);
}

static void StaggeredServoTask(void* arg) {
    // 1. 开机先静默等待 3 秒，确保 WiFi 配网热点 (192.168.4.1) 和 Application 主循环完全启动！
    vTaskDelay(pdMS_TO_TICKS(3000));

    // 2. 在后台任务中错峰归中舵机，绝不阻塞主线程和 Web 配网服务器
    for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
        ServoWriteAngle(i, 90);
        vTaskDelay(pdMS_TO_TICKS(200));
        ServoDetach(i);
        vTaskDelay(pdMS_TO_TICKS(60));
    }

    int active_upper_idx = -1;
    bool gears_sync_active = false;
    int cooldown_ticks = 0;
    int auto_reset_ticks = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(SERVO_STEP_MS));

        if (Application::GetInstance().GetDeviceState() == kDeviceStateStarting ||
            Application::GetInstance().GetDeviceState() == kDeviceStateWifiConfiguring) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (active_upper_idx == -1 && !gears_sync_active) {
            if (cooldown_ticks > 0) {
                cooldown_ticks--;
                continue;
            }
            for (int i = 0; i < SERVO_UPPER_NUM; i++) {
                if (g_servos[i].current_angle != g_servos[i].target_angle) {
                    active_upper_idx = i;
                    g_servos[i].hold_counter = 0;
                    break;
                }
            }
            if (active_upper_idx == -1) {
                if (g_servos[GEAR_LEFT_IDX].current_angle != g_servos[GEAR_LEFT_IDX].target_angle ||
                    g_servos[GEAR_RIGHT_IDX].current_angle != g_servos[GEAR_RIGHT_IDX].target_angle) {
                    gears_sync_active = true;
                    g_servos[GEAR_LEFT_IDX].hold_counter = 0;
                    g_servos[GEAR_RIGHT_IDX].hold_counter = 0;
                }
            }
        }

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
                ServoWriteAngle(GEAR_LEFT_IDX, sL.current_angle);
                ServoWriteAngle(GEAR_RIGHT_IDX, sR.current_angle);
            } else {
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

        // 仅在 Idle 待机状态下检测摸头与左右红外避障
        if (Application::GetInstance().GetDeviceState() != kDeviceStateIdle) {
            continue;
        }

        if (auto_reset_ticks > 0) {
            auto_reset_ticks--;
            if (auto_reset_ticks == 0) {
                for (int i = 0; i < SERVO_UPPER_NUM; i++) {
                    SetServoTargetSafe(i, 90);
                }
                SetGearTrackSync(0, 0);
                if (g_matrix_display) {
                    g_matrix_display->SetGazeHint(0.0f);
                }
                Application::GetInstance().Schedule([]() {
                    auto display = Board::GetInstance().GetDisplay();
                    if (display) {
                        display->SetEmotion("neutral");
                    }
                });
            }
        } else {
            bool touch_active = (gpio_get_level(SENSOR_TOUCH_GPIO) == 1);
            bool left_ir_active = (gpio_get_level(SENSOR_IR_LEFT_GPIO) == 0);
            bool right_ir_active = (gpio_get_level(SENSOR_IR_RIGHT_GPIO) == 0);

            if (touch_active) {
                SetServoTargetSafe(0, 90);
                SetServoTargetSafe(1, 65);
                SetServoTargetSafe(2, 145);
                SetServoTargetSafe(3, 35);
                SetGearTrackSync(50, 0);
                auto_reset_ticks = 75;
                if (g_matrix_display) g_matrix_display->SetGazeHint(0.0f);

                Application::GetInstance().Schedule([]() {
                    auto& app = Application::GetInstance();
                    auto display = Board::GetInstance().GetDisplay();
                    if (display) {
                        display->SetEmotion("happy");
                        display->SetChatMessage("system", "感受到摸头啦~");
                    }
                    if (app.GetDeviceState() == kDeviceStateIdle) {
                        app.ToggleChatState();
                    }
                });
            } else if (left_ir_active && right_ir_active) {
                SetServoTargetSafe(0, 90);
                SetGearTrackSync(-45, 0);
                auto_reset_ticks = 75;
                if (g_matrix_display) g_matrix_display->SetGazeHint(0.0f);

                Application::GetInstance().Schedule([]() {
                    auto display = Board::GetInstance().GetDisplay();
                    if (display) {
                        display->SetEmotion("surprised");
                        display->SetChatMessage("system", "哇！离我太近啦！");
                    }
                });
            } else if (left_ir_active) {
                SetServoTargetSafe(0, 135);
                SetGearTrackSync(45, 2);
                auto_reset_ticks = 75;
                if (g_matrix_display) g_matrix_display->SetGazeHint(-3.2f);

                Application::GetInstance().Schedule([]() {
                    auto display = Board::GetInstance().GetDisplay();
                    if (display) {
                        display->SetEmotion("thinking");
                        display->SetChatMessage("system", "左边有动静 👀");
                    }
                });
            } else if (right_ir_active) {
                SetServoTargetSafe(0, 45);
                SetGearTrackSync(45, 1);
                auto_reset_ticks = 75;
                if (g_matrix_display) g_matrix_display->SetGazeHint(3.2f);

                Application::GetInstance().Schedule([]() {
                    auto display = Board::GetInstance().GetDisplay();
                    if (display) {
                        display->SetEmotion("cool");
                        display->SetChatMessage("system", "右边有动静 👀");
                    }
                });
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

    // 仅注册通道，耗时 0ms（归中动作已移至 StaggeredServoTask 后台任务）
    for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
        ledc_channel_config_t ch_conf = {};
        ch_conf.gpio_num = g_servos[i].pin;
        ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_conf.channel = g_servos[i].channel;
        ch_conf.timer_sel = LEDC_TIMER_1;
        ch_conf.duty = 0;
        ch_conf.hpoint = 0;
        ledc_channel_config(&ch_conf);
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
            int sync_val = properties["gear_sync"].value<int>();
            SetGearTrackSync(sync_val - 90, 0);
            return true;
        });

    McpServer::GetInstance().AddTool("self.face.set_mode",
        "控制200x200居中发光点阵表情屏开关与待机配色。enabled(1=开启点阵表情, 0=显示原生界面)；"
        "color_theme(0=青蓝, 1=暖白, 2=樱花粉, 3=薄荷绿, 4=琥珀金, 5=珊瑚红, 6=柔紫, 7=冰蓝)。",
        PropertyList({
            Property("enabled", kPropertyTypeInteger, 1, 0, 1),
            Property("color_theme", kPropertyTypeInteger, 0, 0, 7)
        }),
        [](const PropertyList& properties) -> ReturnValue {
            if (g_matrix_display) {
                g_matrix_display->SetMatrixEnabled(properties["enabled"].value<int>() != 0);
                g_matrix_display->SetThemeColor(properties["color_theme"].value<int>());
            }
            return true;
        });

    // 以最低优先级 (2) 启动后台任务，绝不抢占 WiFi 配网与语音线程
    xTaskCreate(StaggeredServoTask, "servo_stagger", 4096, nullptr, 2, nullptr);
}

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
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
#ifdef LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        auto* matrix_disp = new GlowMatrixLcdDisplay(
            panel_io, panel,
            DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        g_matrix_display = matrix_disp;
        display_ = matrix_disp;
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

    void InitializeTools() {
        // static LampController lamp(LAMP_GPIO); // 注释掉示例测试灯，释放 GPIO 18 给右侧红外避障模块
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

        // 1. 零延时注册舵机与传感器通道（实际归中在后台 3 秒后执行）
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
                    case 6:
                        SetGearTrackSync(65, 0);
                        break;
                    case 7:
                        SetGearTrackSync(-65, 0);
                        break;
                    case 8:
                        SetGearTrackSync(55, 1);
                        break;
                    case 9:
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
