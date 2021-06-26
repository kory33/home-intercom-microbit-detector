#include <deque>
#include <vector>
#include <algorithm>
#include <cmath>
#include "FrequencyDetector.hpp"

FrequencyDetector::FrequencyDetector(float_t frequency, float_t delay_ms, float_t duration_ms,
                                     float_t frequency_threshold, float_t detection_threshold) :
        delayed_detection_history(std::deque<bool>()),
        detection_history(std::deque<bool>()),
        repacked_buffer(std::vector<float_t>()),
        frequency(frequency),
        frequency_threshold(frequency_threshold),

        delayed_history_length(std::max<uint16_t>((uint16_t) (delay_ms * detection_frame_per_millisecond), 1)),
        detection_history_length(std::max<uint16_t>((uint16_t) (duration_ms * detection_frame_per_millisecond), 1)),
        detection_threshold_integer((uint16_t) ((float_t) detection_history_length * detection_threshold)) {}

bool FrequencyDetector::detect_in_full_repacked_buffer() {
    // We use Goertzel algorithm (https://en.wikipedia.org/wiki/Goertzel_algorithm)
    constexpr auto pi = 3.1415926535898f;

    const auto omega = 2.0f * pi * frequency / sampling_rate;
    const auto cos_omega = cos(omega);
    const auto coefficient = 2 * cos_omega;

    auto s_prev = 0.0;
    auto s_prev_2 = 0.0;

    for (const auto& next_sample : repacked_buffer) {
        const auto s = next_sample + coefficient * s_prev - s_prev_2;
        s_prev_2 = s_prev;
        s_prev = s;
    }

    const auto power = s_prev_2 * s_prev_2 + s_prev * s_prev - coefficient * s_prev * s_prev_2;

    return power > frequency_threshold;
}

void FrequencyDetector::process_buffer(const std::vector <float_t> &input) {
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

bool FrequencyDetector::detected() const {
    if (detection_history.size() < detection_history_length) {
        return false;
    }

    const auto detection_count = std::count(detection_history.cbegin(), detection_history.cend(), true);
    return detection_count >= detection_threshold_integer;
}
