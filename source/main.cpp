#include <memory>
#include <queue>
#include <numeric>
#include <algorithm>
#include "MicroBit.h"

MicroBit uBit;

constexpr int sampling_rate = 11025;

class FrequencyDetector
{
    // internal states of a detector
    std::deque<bool> delayed_detection_history;
    std::deque<bool> detection_history;
    std::vector<float_t> repacked_buffer;

    // configurations
    const float_t frequency;
    const float_t frequency_threshold;

    // we are repacking data from ADC to our own buffer
    static constexpr uint16_t repacked_buffer_size = 256;

    static constexpr float_t detection_frame_per_millisecond =
            ((float_t) sampling_rate / 1000.0) / (float_t) repacked_buffer_size;

    /**
     * the number of detection frames to go back in detection logic
     */
    const uint16_t delayed_history_length;

    /**
     * the number of detection frames to keep for detection
     */
    const uint16_t detection_history_length;

    /**
     * number of frequency-detected frames (within detection_history_length) required to confirm frequency detection
     */
    const uint16_t detection_threshold_integer;

    /**
     * run detection algorithm within repacked buffer with intention to empty the buffer
     */
    bool detect_in_full_repacked_buffer() {
        // We use Goertzel algorithm (https://en.wikipedia.org/wiki/Goertzel_algorithm)
        constexpr auto pi = 3.1415926535898f;

        const auto omega = 2.0f * pi * frequency / sampling_rate;
        const auto cos_omega = cos(omega);
        const auto coefficient = 2 * cos_omega;

        auto s_prev = 0.0f;
        auto s_prev_2 = 0.0f;

        for (const auto& next_sample : repacked_buffer) {
            const auto s = next_sample + coefficient * s_prev - s_prev_2;
            s_prev_2 = s_prev;
            s_prev = s;
        }

        const auto power = s_prev_2 * s_prev_2 + s_prev * s_prev - coefficient * s_prev * s_prev_2;

//        uBit.serial.printf("%d\t", (uint16_t) (power * 1000));

        return power > frequency_threshold;
    }

public:
    /**
     * @param frequency the value of the frequency (in Hz) to detect
     * @param delay_ms approximate delay (in ms) to introduce in the detection
     * @param duration_ms approximate duration (in ms) in which the specified frequency component should be above threshold
     * @param frequency_threshold threshold to admit detection in a frame
     * @param detection_threshold rate of detected frame to non-detected frames to confirm detection (default 0.9)
     */
    FrequencyDetector(float_t frequency, float_t delay_ms, float_t duration_ms,
                      float_t frequency_threshold, float_t detection_threshold = 0.9f) :
            delayed_detection_history(std::deque<bool>()),
            detection_history(std::deque<bool>()),
            repacked_buffer(std::vector<float_t>()),
            frequency(frequency),
            frequency_threshold(frequency_threshold),

            delayed_history_length(max((uint16_t) (delay_ms * detection_frame_per_millisecond), 1)),
            detection_history_length(max((uint16_t) (duration_ms * detection_frame_per_millisecond), 1)),
            detection_threshold_integer((uint16_t) ((float_t) detection_history_length * detection_threshold)) {}

    void process_buffer(const std::vector<float_t>& input) {
        for (const auto& v: input) {
            repacked_buffer.push_back(v);

            if (repacked_buffer.size() >= repacked_buffer_size) {
                delayed_detection_history.push_front(detect_in_full_repacked_buffer());
                repacked_buffer.clear();
            }
        }

        // move excessive data from delayed detection history to detection history
        while (delayed_detection_history.size() > delayed_history_length) {
            detection_history.push_front(delayed_detection_history.back());
            delayed_detection_history.pop_back();
        }

        // delete excessive data in detection history
        while (detection_history.size() > detection_history_length) {
            detection_history.pop_back();
        }
    }

    [[nodiscard]] bool detected() const {
        if (detection_history.size() < detection_history_length) {
            return false;
        }

        const auto detection_count =
                std::count(detection_history.cbegin(), detection_history.cend(), true);

        return detection_count >= detection_threshold_integer;
    }
};

class Printer : public DataSink
{
    DataSource &upstream;
    Serial &serial;
    std::array<FrequencyDetector, 4> frequencyDetectors;
public:
    Printer(DataSource &upstream, Serial &serial):
        upstream(upstream),
        serial(serial),
        /*
         * My intercom has the following ring tone pattern:
         *  - 650Hz for 750ms
         *  - 510Hz for 1250ms
         *  - 650Hz for 750ms
         *  - 510Hz for 1250ms
         *
         * To detect this pattern, we listen on
         *  - 650Hz for 500ms, with 3100ms delay
         *  - 510Hz for 750ms, with 2100ms delay
         *  - 650Hz for 500ms, with 1100ms delay
         *  - 510Hz for 750ms, with 0ms delay
         *
         * I expect all of these detectors to detect sound by the time last 510Hz sound has played for 1000ms,
         * so there is enough time window for each detectors to catch intercom sound.
         *
         * Roughly these detection windows look like this:
         *
         * 650Hz: ...-|<= 750ms =>|-------------------|<= 750ms =>|---------------------...
         *
         * 510Hz: ...-------------|<===== 1250ms ====>|-----------|<===== 1250ms ====>|-...
         *
         *               |<=d1=>|   |<== d 2 ==>|        |<=d3=>|     |<== d 4 ==>|
         */
        frequencyDetectors({
            FrequencyDetector(650.0, 3100.0, 500.0, 5.0, 0.8),
            FrequencyDetector(510.0, 2100.0, 750.0, 0.5, 0.8),
            FrequencyDetector(650.0, 1100.0, 500.0, 5.0, 0.8),
            FrequencyDetector(510.0, 0.0, 750.0, 0.5, 0.8)
        }) {}

    int pullRequest() override {
        const auto buffer = upstream.pull();
        const auto decoded_buffer_size = buffer.length() / 2;

        std::vector<int16_t> decoded_values;
        std::vector<float_t> float_values;

        for (volatile int i = 0; i < buffer.length(); i += 2) {
            const auto lower = (uint16_t) buffer[i];
            const auto upper = (uint16_t) buffer[i + 1];
            const auto sum = lower + (upper << 8);

            decoded_values.push_back((int16_t) sum);
        }

        // take average for normalization
        const auto sum = std::accumulate(decoded_values.cbegin(), decoded_values.cend(), (int32_t) 0);
        const auto average = (int16_t) (sum / decoded_buffer_size);

        for (const auto& v : decoded_values) {
            const auto normalized_value = (float_t)(v - average) / (float_t)(1 << 12);
            float_values.push_back(normalized_value);
        }

        for (auto& d : frequencyDetectors) {
            d.process_buffer(float_values);

            if (d.detected()) {
                serial.printf("*\t");
            } else {
                serial.printf("_\t");
            }
        }
        serial.printf("\n");

        return DEVICE_OK;
    }
};

int main() {
    uBit.init();

    uBit.serial.printf("\033[2JStarting micro:bit...\n");

    // activate microphone
    uBit.io.runmic.setDigitalValue(1);
    uBit.io.runmic.setHighDrive(true);

    auto mic_channel = std::unique_ptr<NRF52ADCChannel>(uBit.adc.getChannel(uBit.io.microphone));
    mic_channel->setGain(7, 0);
    mic_channel->output.setBlocking(true);

    auto sink = std::make_unique<Printer>(mic_channel->output, uBit.serial);
    mic_channel->output.connect(*sink);

    #pragma ide diagnostic ignored "EndlessLoop"
    while (true) {
        uBit.wait(10);
    }
}
