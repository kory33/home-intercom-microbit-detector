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

void MessagingSurrogate::init() {
    executeAndWaitCommand(CustomCommand::ConfirmStartup);
}

void MessagingSurrogate::onSoundDetectedEvent() {
    executeAndWaitCommand(CustomCommand::NotifySoundDetection);
}

void MessagingSurrogate::beginCommandLoop() {
    while (true) {
        if (!command_queue.empty()) {
            const auto command = command_queue.front();
            const auto expected = std::string("COMPLETED:") + static_cast<char>(command.command);

            sendCommand(command.command);

            while (true) {
                // if we have got some response from the module regarding the command
                if (expectToRead(expected)) break;

                sendCommand(CustomCommand::QueryPreviousResult);
            }

            event_bus.send(Event(command_completed_event_id, command.timestamp));

            sendCommand(CustomCommand::ForgetPreviousResult);
        }

        schedule();
    }
}

void MessagingSurrogate::beginPingRemoteLoop() {
    constexpr auto ping_interval_ms = 500;

    while (true) {
        executeAndWaitCommand(CustomCommand::PingRemote);
        fiber_sleep(ping_interval_ms);
    }
}


void surrogateCommandLoopEntryPoint(void* surrogate) {
    static_cast<MessagingSurrogate*>(surrogate)->beginCommandLoop();
}

void esp_01s::beginSurrogateCommandLoopUsing(MessagingSurrogate* surrogate) {
    codal::create_fiber(surrogateCommandLoopEntryPoint, surrogate);
}


void pingRemoteLoopEntryPoint(void* surrogate) {
    static_cast<MessagingSurrogate*>(surrogate)->beginPingRemoteLoop();
}

void esp_01s::beginPingRemoteLoopUsing(MessagingSurrogate* surrogate) {
    codal::create_fiber(pingRemoteLoopEntryPoint, surrogate);
}
