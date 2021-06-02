#include <memory>
#include <queue>
#include <numeric>
#include <algorithm>
#include "MicroBit.h"

class Printer : public DataSink
{
    DataSource &upstream;
    Serial &serial;
public:
    Printer(DataSource &upstream, Serial &serial): upstream(upstream), serial(serial) {
    }

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

        for (const auto& v : float_values) {
            if (v > 0.0f) serial.printf("*");
            else serial.printf("_");
        }

        return DEVICE_OK;
    }
};

int main() {
    MicroBit uBit;

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
