#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "driver/i2s_std.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

namespace {

constexpr char TAG[] = "I2S_LOWPASS";

constexpr std::uint32_t SAMPLE_RATE = 48000;

// Change this value in the IDE.
// Valid range: greater than 0 and less than 24000 Hz.
constexpr float CUTOFF_HZ = 300.0f;

constexpr std::size_t CHANNEL_COUNT = 2;
constexpr std::size_t BLOCK_FRAMES = 64;
constexpr std::size_t BLOCK_SAMPLES =
    BLOCK_FRAMES * CHANNEL_COUNT;

constexpr float PI = 3.14159265358979323846f;

// Shared clocks
constexpr gpio_num_t PIN_MCLK = GPIO_NUM_0;
constexpr gpio_num_t PIN_LRCK = GPIO_NUM_18;
constexpr gpio_num_t PIN_BCLK = GPIO_NUM_5;

// ESP32 output -> Pmod DAC SDIN
constexpr gpio_num_t PIN_DOUT = GPIO_NUM_19;

// Pmod ADC SDOUT -> ESP32 input
constexpr gpio_num_t PIN_DIN = GPIO_NUM_17;

static_assert(
    CUTOFF_HZ > 0.0f &&
    CUTOFF_HZ < static_cast<float>(SAMPLE_RATE) / 2.0f,
    "CUTOFF_HZ must be between 0 and the Nyquist frequency"
);

i2s_chan_handle_t tx_channel = nullptr;
i2s_chan_handle_t rx_channel = nullptr;

alignas(4) std::int32_t audio_buffer[BLOCK_SAMPLES] = {};

/*
 * Second-order Butterworth low-pass biquad.
 *
 * Its response is approximately flat below the cutoff,
 * -3 dB at the cutoff, and rolls off at 12 dB/octave.
 */
class ButterworthLowPass {
public:
    ButterworthLowPass(float cutoff_hz, float sample_rate_hz)
    {
        constexpr float Q = 0.70710678118f;

        const float omega =
            2.0f * PI * cutoff_hz / sample_rate_hz;

        const float cosine = std::cos(omega);
        const float sine = std::sin(omega);
        const float alpha = sine / (2.0f * Q);

        const float a0 = 1.0f + alpha;

        b0_ = ((1.0f - cosine) * 0.5f) / a0;
        b1_ = (1.0f - cosine) / a0;
        b2_ = ((1.0f - cosine) * 0.5f) / a0;

        a1_ = (-2.0f * cosine) / a0;
        a2_ = (1.0f - alpha) / a0;
    }

    float process(float input)
    {
        // Transposed Direct Form II
        const float output = b0_ * input + state_1_;

        state_1_ =
            b1_ * input -
            a1_ * output +
            state_2_;

        state_2_ =
            b2_ * input -
            a2_ * output;

        return output;
    }

private:
    float b0_ = 0.0f;
    float b1_ = 0.0f;
    float b2_ = 0.0f;
    float a1_ = 0.0f;
    float a2_ = 0.0f;

    float state_1_ = 0.0f;
    float state_2_ = 0.0f;
};

void initialize_i2s()
{
    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_0,
            I2S_ROLE_MASTER
        );

    channel_config.dma_desc_num = 8;
    channel_config.dma_frame_num = BLOCK_FRAMES;

    ESP_ERROR_CHECK(
        i2s_new_channel(
            &channel_config,
            &tx_channel,
            &rx_channel
        )
    );

    i2s_std_config_t standard_config{};

    standard_config.clk_cfg =
        I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE);

    standard_config.clk_cfg.mclk_multiple =
        I2S_MCLK_MULTIPLE_256;

    standard_config.slot_cfg =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_STEREO
        );

    standard_config.slot_cfg.slot_bit_width =
        I2S_SLOT_BIT_WIDTH_32BIT;

    standard_config.slot_cfg.ws_width = 32;

    standard_config.gpio_cfg.mclk = PIN_MCLK;
    standard_config.gpio_cfg.bclk = PIN_BCLK;
    standard_config.gpio_cfg.ws = PIN_LRCK;
    standard_config.gpio_cfg.dout = PIN_DOUT;
    standard_config.gpio_cfg.din = PIN_DIN;

    standard_config.gpio_cfg.invert_flags.mclk_inv = false;
    standard_config.gpio_cfg.invert_flags.bclk_inv = false;
    standard_config.gpio_cfg.invert_flags.ws_inv = false;

    ESP_ERROR_CHECK(
        i2s_channel_init_std_mode(
            tx_channel,
            &standard_config
        )
    );

    ESP_ERROR_CHECK(
        i2s_channel_init_std_mode(
            rx_channel,
            &standard_config
        )
    );

    ESP_ERROR_CHECK(i2s_channel_enable(tx_channel));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_channel));
}

std::int32_t process_sample(
    std::int32_t input,
    ButterworthLowPass& filter
)
{
    constexpr float INPUT_SCALE =
        1.0f / 2147483648.0f;

    constexpr float OUTPUT_SCALE =
        2147483647.0f;

    const float normalized_input =
        static_cast<float>(input) * INPUT_SCALE;

    const float filtered =
        filter.process(normalized_input);

    const float limited =
        std::clamp(filtered, -1.0f, 1.0f);

    return static_cast<std::int32_t>(
        limited * OUTPUT_SCALE
    );
}

} // namespace

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Initializing filtered passthrough");
    ESP_LOGI(TAG, "C++ value: %ld", __cplusplus);
    ESP_LOGI(TAG, "Sample rate: %lu Hz",
             static_cast<unsigned long>(SAMPLE_RATE));
    ESP_LOGI(TAG, "Low-pass cutoff: %.1f Hz",
             static_cast<double>(CUTOFF_HZ));

    initialize_i2s();

    ButterworthLowPass left_filter(
        CUTOFF_HZ,
        static_cast<float>(SAMPLE_RATE)
    );

    ButterworthLowPass right_filter(
        CUTOFF_HZ,
        static_cast<float>(SAMPLE_RATE)
    );

    ESP_LOGI(TAG, "Filtered passthrough started");

    while (true) {
        std::size_t bytes_read = 0;

        ESP_ERROR_CHECK(
            i2s_channel_read(
                rx_channel,
                audio_buffer,
                sizeof(audio_buffer),
                &bytes_read,
                portMAX_DELAY
            )
        );

        const std::size_t samples_read =
            bytes_read / sizeof(audio_buffer[0]);

        // Interleaved stereo: left, right, left, right...
        for (std::size_t index = 0;
             index + 1 < samples_read;
             index += 2) {

            audio_buffer[index] =
                process_sample(
                    audio_buffer[index],
                    left_filter
                );

            audio_buffer[index + 1] =
                process_sample(
                    audio_buffer[index + 1],
                    right_filter
                );
        }

        std::size_t total_written = 0;

        while (total_written < bytes_read) {
            std::size_t bytes_written = 0;

            ESP_ERROR_CHECK(
                i2s_channel_write(
                    tx_channel,
                    reinterpret_cast<std::uint8_t*>(
                        audio_buffer
                    ) + total_written,
                    bytes_read - total_written,
                    &bytes_written,
                    portMAX_DELAY
                )
            );

            total_written += bytes_written;
        }
    }
}