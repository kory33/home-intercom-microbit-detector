#include <memory>
#include <queue>
#include <numeric>
#include <algorithm>
#include <string>
#include "MicroBit.h"

static constexpr auto sampling_rate = 11025;

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

        const auto detection_count = std::count(detection_history.cbegin(), detection_history.cend(), true);
        return detection_count >= detection_threshold_integer;
    }
};

class FrequencyDetectorController : public DataSink
{
    // we should not invoke callback for approximately 5 seconds after first detecting a pattern
    static const auto detection_cool_down_tick = 40 * 5;

    DataSource &upstream;
    Serial &serial;
    std::function<void()> callback;
    std::array<FrequencyDetector, 4> frequencyDetectors;
    uint16_t ticks_to_wait_before_detection = 0;
public:
    FrequencyDetectorController(DataSource &upstream, Serial &serial, std::function<void()> onDetectedEvent):
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

        if (ticks_to_wait_before_detection == 0 &&
            std::all_of(frequencyDetectors.begin(), frequencyDetectors.end(), [](auto d) { return d.detected(); })) {

            ticks_to_wait_before_detection = detection_cool_down_tick;
            invoke(this->callback);
        }

        if (ticks_to_wait_before_detection > 0) ticks_to_wait_before_detection -= 1;

        return DEVICE_OK;
    }
};

enum class Esp01SCustomCommand : char {
    ConfirmStartup = 'C',
    PingRemote = 'R',
    NotifySoundDetection = 'N',
    QueryPreviousResult = 'P',
    ForgetPreviousResult = 'Q',
};

static uint16_t command_timestamp = 0;
class TimeStampedCommand {
public:
    const Esp01SCustomCommand command;
    const uint16_t timestamp;

    explicit TimeStampedCommand(Esp01SCustomCommand command) :
      command(command),
      timestamp(command_timestamp) {};
};

/**
 * The class of objects acting as a middle-man of communication with ESP-01S module.
 */
class Esp01SMessagingSurrogate
{
    static const uint16_t command_completed_event_id = 0x5000;

    MessageBus event_bus;
    const std::unique_ptr<MicroBitSerial> serial;
    std::queue<TimeStampedCommand> command_queue;

    void flushRx() {
        serial->clearRxBuffer();
    }

    void executeAndWaitCommand(Esp01SCustomCommand command) {
        const auto timestamped = TimeStampedCommand(command);

        command_queue.push(timestamped);

        fiber_wait_for_event(command_completed_event_id, timestamped.timestamp);
    }

    void sendCommand(Esp01SCustomCommand command) {
        serial->putc(static_cast<char>(command));
    }

    [[nodiscard]] std::string waitAndReadAsync(unsigned int size) {
        constexpr auto sleep_ms = 100;

        fiber_sleep(sleep_ms);

        const auto read_result = serial->read((int) size, ASYNC);

        return std::string(read_result.toCharArray());
    }

    /**
     * Expect to read the string immediately after 100ms has passed, and then clear RX.
     * @return true iff we have found the expected string on RX after sleeping for 100ms.
     */
    bool expectToRead(const std::string& expected) {
        const auto response = waitAndReadAsync(expected.length());

        flushRx();

        return response == expected;
    }

public:
    Esp01SMessagingSurrogate(const MessageBus& message_bus,
                             NRF52Pin tx,
                             NRF52Pin rx,
                             uint8_t rxBufferSize = CODAL_SERIAL_DEFAULT_BUFFER_SIZE,
                             uint8_t txBufferSize = CODAL_SERIAL_DEFAULT_BUFFER_SIZE):
        event_bus(message_bus),
        serial(std::make_unique<MicroBitSerial>(tx, rx, rxBufferSize, txBufferSize)),
        command_queue() {};

    /**
     * Wait until ESP-01S module is able to answer us.
     */
    void init() {
        executeAndWaitCommand(Esp01SCustomCommand::ConfirmStartup);
    }

    void onSoundDetectedEvent() {
        executeAndWaitCommand(Esp01SCustomCommand::NotifySoundDetection);
    }

    /**
     * Run the command-loop. The behaviour is undefined if this function is executed more than once.
     */
    [[noreturn]] void beginCommandLoop() {
        while (true) {
            if (!command_queue.empty()) {
                const auto command = command_queue.front();
                const auto expected = std::string("COMPLETED:") + static_cast<char>(command.command);

                sendCommand(command.command);

                while (true) {
                    // if we have got some response from the module regarding the command
                    if (expectToRead(expected)) break;

                    sendCommand(Esp01SCustomCommand::QueryPreviousResult);
                }

                event_bus.send(Event(command_completed_event_id, command.timestamp));

                sendCommand(Esp01SCustomCommand::ForgetPreviousResult);
            }

            schedule();
        }
    }

    /**
     * Run the task to ping remote every half a second.
     */
    [[noreturn]] void beginPingRemoteLoop() {
        constexpr auto ping_interval_ms = 500;

        while (true) {
            executeAndWaitCommand(Esp01SCustomCommand::PingRemote);
            fiber_sleep(ping_interval_ms);
        }
    }
};

[[noreturn]] void beginSurrogateCommandLoop(void* surrogate) {
    static_cast<Esp01SMessagingSurrogate*>(surrogate)->beginCommandLoop();
}

[[noreturn]] void beginPingRemoteLoop(void* surrogate) {
    static_cast<Esp01SMessagingSurrogate*>(surrogate)->beginPingRemoteLoop();
}

int main() {
    {
        auto uBit = std::make_unique<MicroBit>();

        uBit->init();

        uBit->serial.printf("\033[2JStarted micro:bit!\n");

        uBit->serial.printf("Setting up ESP-01...\n");

        auto esp01sSurrogate = std::make_shared<Esp01SMessagingSurrogate>(
                uBit->messageBus,
                uBit->io.P14, uBit->io.P13, 32, 32
        );

        esp01sSurrogate->init();

        uBit->serial.printf("Setting up microphone...\n");

        // activate microphone
        uBit->io.runmic.setDigitalValue(1);
        uBit->io.runmic.setHighDrive(true);

        auto mic_channel = std::unique_ptr<NRF52ADCChannel>(uBit->adc.getChannel(uBit->io.microphone));
        mic_channel->setGain(7, 0);
        mic_channel->output.setBlocking(true);

        auto sink = std::make_unique<FrequencyDetectorController>(
                mic_channel->output, uBit->serial,
                [esp01sSurrogate]() { esp01sSurrogate->onSoundDetectedEvent(); }
        );

        mic_channel->output.connect(*sink);

        codal::create_fiber(beginSurrogateCommandLoop, esp01sSurrogate.get());
        codal::create_fiber(beginPingRemoteLoop, esp01sSurrogate.get());
    }

    #pragma ide diagnostic ignored "EndlessLoop"
    while (true) {
        schedule();
    }
}
