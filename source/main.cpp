#include <memory>
#include <algorithm>
#include "MicroBit.h"

#include "freq_detector/FrequencyDetectorController.hpp"
#include "esp01s/CustomCommand.hpp"
#include "esp01s/MessagingSurrogate.hpp"

[[noreturn]] void schedule_forever() {
    while (true) {
        schedule();
    }
}

int main() {
    const auto uBit = std::make_unique<MicroBit>();

    uBit->init();

    uBit->serial.printf("\033[2JStarted micro:bit!\n");

    uBit->serial.printf("Setting up ESP-01...\n");

    const auto esp01sSurrogate = new esp_01s::MessagingSurrogate(
            uBit->messageBus,
            uBit->io.P14, uBit->io.P13, 32, 32
    );

    esp01sSurrogate->init();

    uBit->serial.printf("Setting up microphone...\n");

    // activate microphone
    uBit->io.runmic.setDigitalValue(1);
    uBit->io.runmic.setHighDrive(true);

    const auto mic_channel = std::unique_ptr<NRF52ADCChannel>(uBit->adc.getChannel(uBit->io.microphone));
    mic_channel->setGain(7, 0);
    mic_channel->output.setBlocking(true);

    const auto sink = std::make_unique<FrequencyDetectorController>(
            mic_channel->output, uBit->serial,
            [esp01sSurrogate]() { esp01sSurrogate->asyncNotifySoundDetection(); }
    );

    mic_channel->output.connect(*sink);

    schedule_forever();
}
