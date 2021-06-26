#pragma once

#include <cstdint>
#include <deque>

class FrequencyDetector
{
    // internal states of a detector
    std::deque<bool> delayed_detection_history;
    std::deque<bool> detection_history;
    std::vector<float_t> repacked_buffer;

    // configurations
    const float_t frequency;
    const float_t frequency_threshold;

    static constexpr auto sampling_rate = 11025;

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
    bool detect_in_full_repacked_buffer();

public:
    /**
     * @param frequency the value of the frequency (in Hz) to detect
     * @param delay_ms approximate delay (in ms) to introduce in the detection
     * @param duration_ms approximate duration (in ms) in which the specified frequency component should be above threshold
     * @param frequency_threshold threshold to admit detection in a frame
     * @param detection_threshold rate of detected frame to non-detected frames to confirm detection (default 0.9)
     */
    FrequencyDetector(float_t frequency, float_t delay_ms, float_t duration_ms,
                      float_t frequency_threshold, float_t detection_threshold = 0.9f);

    void process_buffer(const std::vector<float_t>& input);

    [[nodiscard]] bool detected() const;
};
