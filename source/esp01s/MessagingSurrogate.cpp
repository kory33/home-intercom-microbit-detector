#include "MessagingSurrogate.hpp"

using namespace esp_01s;

void MessagingSurrogate::flushRx() {
    serial->clearRxBuffer();
}

void MessagingSurrogate::executeAndWaitCommand(CustomCommand command) {
    const auto timestamped = TimeStampedCommand(command);

    command_queue.push(timestamped);

    fiber_wait_for_event(command_completed_event_id, timestamped.timestamp);
}

void MessagingSurrogate::sendCommand(CustomCommand command) {
    serial->putc(static_cast<char>(command));
}

std::string MessagingSurrogate::waitAndReadAsync(unsigned int size) {
    constexpr auto sleep_ms = 100;

    fiber_sleep(sleep_ms);

    const auto read_result = serial->read((int) size, ASYNC);

    return std::string(read_result.toCharArray());
}

bool MessagingSurrogate::expectToRead(const std::string &expected) {
    const auto response = waitAndReadAsync(expected.length());

    flushRx();

    return response == expected;
}

MessagingSurrogate::MessagingSurrogate(const MessageBus &message_bus, NRF52Pin tx, NRF52Pin rx,
                                                   uint8_t rxBufferSize, uint8_t txBufferSize) :
        event_bus(message_bus),
        serial(std::make_unique<MicroBitSerial>(tx, rx, rxBufferSize, txBufferSize)),
        command_queue() {}

void MessagingSurrogate::onSoundDetectedEvent() {
    executeAndWaitCommand(CustomCommand::NotifySoundDetection);
}

void MessagingSurrogate::beginCommandLoop() {
    while (true) {
        if (!command_queue.empty()) {
            const auto command = command_queue.front();
            const auto expected = std::string("DONE:") + static_cast<char>(command.command);

            while (true) {
                // these commands are always idempotent
                sendCommand(command.command);

                // if we have got some response from the module regarding the command
                if (expectToRead(expected)) break;
            }

            event_bus.send(Event(command_completed_event_id, command.timestamp));

            sendCommand(CustomCommand::GetReadyForNextCommand);
        }

        schedule();
    }
}

void MessagingSurrogate::beginPingRemoteLoop() {
    /**
     * SSL handshaking on ESP01S module takes substantial time (not exceeding 10s)
     * so ping every 20s approximately (excluding command execution time)
     *
     * We should probably raise an alert on the server-side if we do not see a ping for 1 minute.
     */
    constexpr auto ping_interval_ms = 20000;

    while (true) {
        executeAndWaitCommand(CustomCommand::PingRemote);
        fiber_sleep(ping_interval_ms);
    }
}

void surrogateCommandLoopEntryPoint(void* surrogate) {
    static_cast<MessagingSurrogate*>(surrogate)->beginCommandLoop();
}

void beginSurrogateCommandLoopUsing(MessagingSurrogate* surrogate) {
    codal::create_fiber(surrogateCommandLoopEntryPoint, surrogate);
}

void pingRemoteLoopEntryPoint(void* surrogate) {
    static_cast<MessagingSurrogate*>(surrogate)->beginPingRemoteLoop();
}

void beginPingRemoteLoopUsing(MessagingSurrogate* surrogate) {
    codal::create_fiber(pingRemoteLoopEntryPoint, surrogate);
}

void MessagingSurrogate::init() {
    // this must be before we wait for ConfirmStartup because command-loop is initially not running
    beginSurrogateCommandLoopUsing(this);

    executeAndWaitCommand(CustomCommand::ConfirmStartup);

    beginPingRemoteLoopUsing(this);
}
