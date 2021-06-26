#include <functional>
#include <cmath>
#include <numeric>
#include <ErrorNo.h>
#include "FrequencyDetectorController.hpp"

FrequencyDetectorController::FrequencyDetectorController(codal::DataSource &upstream, codal::Serial &serial,
                                                         std::function<void()> onDetectedEvent) :
        upstream(upstream),
        serial(serial),
        callback(std::move(onDetectedEvent)),
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

int FrequencyDetectorController::pullRequest() {
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

    if (ticks_to_wait_before_detection == 0 &&
        std::all_of(frequencyDetectors.begin(), frequencyDetectors.end(), [](auto d) { return d.detected(); })) {

        ticks_to_wait_before_detection = detection_cool_down_tick;
        invoke(this->callback);
    }

    if (ticks_to_wait_before_detection > 0) ticks_to_wait_before_detection -= 1;

    return DEVICE_OK;
}
