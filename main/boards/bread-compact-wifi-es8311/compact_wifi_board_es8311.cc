#include "wifi_board.h"
#include "codecs/es8311_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "CompactWifiEs8311Board"

class CompactWifiEs8311Board : public WifiBoard {
private:
    // ES8311 Audio Codec I2C 总线
    i2c_master_bus_handle_t codec_i2c_bus_;
    // OLED Display I2C 总线 (如果OLED和ES8311共用I2C,可以合并)
    i2c_master_bus_handle_t display_i2c_bus_;
    
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;

    // 初始化 ES8311 的 I2C 总线
    void InitializeCodecI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &audio_i2c_bus_));
        ESP_LOGI(TAG, "Audio I2C initialized (SDA=%d, SCL=%d)", AUDIO_CODEC_I2C_SDA_PIN, AUDIO_CODEC_I2C_SCL_PIN);
        // 扫描确认 ES8311 是否在线(地址 0x18),用于快速定位硬件接线问题
        if (i2c_master_probe(audio_i2c_bus_, AUDIO_CODEC_ES8311_ADDR, 100) == ESP_OK) {
            ESP_LOGI(TAG, "ES8311 found on I2C bus (addr=0x%02X)", AUDIO_CODEC_ES8311_ADDR);
        } else {
            ESP_LOGE(TAG, "ES8311 NOT found on I2C bus (addr=0x%02X)! Check SDA/SCL wiring and power", AUDIO_CODEC_ES8311_ADDR);
        }
    }

    // 初始化 OLED 显示屏的 I2C 总线
    // 如果 OLED 和 ES8311 共用相同的 SDA/SCL 引脚,可以复用 audio_i2c_bus_
    void InitializeDisplayI2c() {
        // 检查是否可以复用音频的I2C总线
        if (DISPLAY_I2C_SDA_PIN == AUDIO_CODEC_I2C_SDA_PIN && 
            DISPLAY_I2C_SCL_PIN == AUDIO_CODEC_I2C_SCL_PIN) {
            ESP_LOGI(TAG, "Display I2C will share bus with audio codec");
            return;
        }
        
        i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_1,
            .sda_io_num = DISPLAY_I2C_SDA_PIN,
            .scl_io_num = DISPLAY_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
        ESP_LOGI(TAG, "Display I2C initialized (SDA=%d, SCL=%d)", DISPLAY_I2C_SDA_PIN, DISPLAY_I2C_SCL_PIN);
    }
    
    // 获取OLED使用的I2C总线
    i2c_master_bus_handle_t GetDisplayI2cBus() {
        if (DISPLAY_I2C_SDA_PIN == AUDIO_CODEC_I2C_SDA_PIN && 
            DISPLAY_I2C_SCL_PIN == AUDIO_CODEC_I2C_SCL_PIN) {
            return audio_i2c_bus_;
        }
        return display_i2c_bus_;
    }

    void InitializeSsd1306Display() {
        i2c_master_bus_handle_t i2c_bus = GetDisplayI2cBus();

        // 扫描确认 OLED 是否在线(常见地址 0x3C / 0x3D),用于快速定位接线问题
        if (i2c_master_probe(i2c_bus, 0x3C, 100) == ESP_OK) {
            ESP_LOGI(TAG, "OLED found at I2C addr 0x3C");
        } else if (i2c_master_probe(i2c_bus, 0x3D, 100) == ESP_OK) {
            ESP_LOGI(TAG, "OLED found at I2C addr 0x3D");
        } else {
            ESP_LOGE(TAG, "OLED NOT found on I2C bus! Check SDA/SCL wiring (DISPLAY_SDA=%d, SCL=%d) and 3.3V power",
                     DISPLAY_I2C_SDA_PIN, DISPLAY_I2C_SCL_PIN);
        }

        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .scl_speed_hz = 400 * 1000,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
        };

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install SSD1306 driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

#ifdef SH1106
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
#endif
        ESP_LOGI(TAG, "OLED driver installed");

        // Reset the display
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize display");
            display_ = new NoDisplay();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));

        // Set the display to on
        ESP_LOGI(TAG, "Turning display on");
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
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
        touch_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        touch_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    // 物联网初始化，逐步迁移到 MCP 协议
    void InitializeTools() {
        static LampController lamp(LAMP_GPIO);
    }

public:
    CompactWifiEs8311Board() :
        boot_button_(BOOT_BUTTON_GPIO),
        touch_button_(TOUCH_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        // 先初始化 ES8311 的 I2C 总线
        InitializeAudioI2c();
        // 再初始化 OLED 显示
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
        InitializeTools();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(
            codec_i2c_bus_,           // I2C master bus handle
            I2C_NUM_0,                // I2C port (与上面保持一致)
            AUDIO_INPUT_SAMPLE_RATE,  // 输入采样率
            AUDIO_OUTPUT_SAMPLE_RATE, // 输出采样率 (必须等于输入)
            AUDIO_I2S_GPIO_MCLK,      // MCLK
            AUDIO_I2S_GPIO_BCLK,      // BCLK (SCLK)
            AUDIO_I2S_GPIO_WS,       // WS (LRCK)
            AUDIO_I2S_GPIO_DOUT,      // DOUT (ESP32 -> ES8311 DIN, 播放)
            AUDIO_I2S_GPIO_DIN,       // DIN (ESP32 <- ES8311 DOUT, 录音)
            AUDIO_CODEC_PA_PIN,       // PA control pin (如需要功放使能)
            AUDIO_CODEC_ES8311_ADDR); // ES8311 I2C address
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(CompactWifiEs8311Board);
