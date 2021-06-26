#pragma once

#include <DataStream.h>
#include <Serial.h>
#include "FrequencyDetector.hpp"

class FrequencyDetectorController : public codal::DataSink
{
    // we should not invoke callback for approximately 5 seconds after first detecting a pattern
    static const auto detection_cool_down_tick = 40 * 5;

    codal::DataSource &upstream;
    codal::Serial &serial;
    std::function<void()> callback;
    std::array<FrequencyDetector, 4> frequencyDetectors;
    uint16_t ticks_to_wait_before_detection = 0;

public:
    FrequencyDetectorController(codal::DataSource &upstream, codal::Serial &serial, std::function<void()> onDetectedEvent);

    int pullRequest() override;
};
