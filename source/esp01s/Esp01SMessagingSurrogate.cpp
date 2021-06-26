#include "Esp01SMessagingSurrogate.hpp"

void Esp01SMessagingSurrogate::flushRx() {
    serial->clearRxBuffer();
}

void Esp01SMessagingSurrogate::executeAndWaitCommand(Esp01SCustomCommand command) {
    const auto timestamped = TimeStampedCommand(command);

    command_queue.push(timestamped);

    fiber_wait_for_event(command_completed_event_id, timestamped.timestamp);
}

void Esp01SMessagingSurrogate::sendCommand(Esp01SCustomCommand command) {
    serial->putc(static_cast<char>(command));
}

std::string Esp01SMessagingSurrogate::waitAndReadAsync(unsigned int size) {
    constexpr auto sleep_ms = 100;

    fiber_sleep(sleep_ms);

    const auto read_result = serial->read((int) size, ASYNC);

    return std::string(read_result.toCharArray());
}

bool Esp01SMessagingSurrogate::expectToRead(const std::string &expected) {
    const auto response = waitAndReadAsync(expected.length());

    flushRx();

    return response == expected;
}

Esp01SMessagingSurrogate::Esp01SMessagingSurrogate(const MessageBus &message_bus, NRF52Pin tx, NRF52Pin rx,
                                                   uint8_t rxBufferSize, uint8_t txBufferSize) :
        event_bus(message_bus),
        serial(std::make_unique<MicroBitSerial>(tx, rx, rxBufferSize, txBufferSize)),
        command_queue() {}

void Esp01SMessagingSurrogate::init() {
    executeAndWaitCommand(Esp01SCustomCommand::ConfirmStartup);
}

void Esp01SMessagingSurrogate::onSoundDetectedEvent() {
    executeAndWaitCommand(Esp01SCustomCommand::NotifySoundDetection);
}

void Esp01SMessagingSurrogate::beginCommandLoop() {
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

void Esp01SMessagingSurrogate::beginPingRemoteLoop() {
    constexpr auto ping_interval_ms = 500;

    while (true) {
        executeAndWaitCommand(Esp01SCustomCommand::PingRemote);
        fiber_sleep(ping_interval_ms);
    }
}


void surrogateCommandLoopEntryPoint(void* surrogate) {
    static_cast<Esp01SMessagingSurrogate*>(surrogate)->beginCommandLoop();
}

void beginSurrogateCommandLoopUsing(Esp01SMessagingSurrogate* surrogate) {
    codal::create_fiber(surrogateCommandLoopEntryPoint, surrogate);
}


void pingRemoteLoopEntryPoint(void* surrogate) {
    static_cast<Esp01SMessagingSurrogate*>(surrogate)->beginPingRemoteLoop();
}

void beginPingRemoteLoopUsing(Esp01SMessagingSurrogate* surrogate) {
    codal::create_fiber(pingRemoteLoopEntryPoint, surrogate);
}
