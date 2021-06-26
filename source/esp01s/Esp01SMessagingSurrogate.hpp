#pragma once

#include <cstdint>
#include <memory>
#include <MessageBus.h>
#include <queue>
#include <string>
#include <MicroBitSerial.h>
#include "Esp01SCommand.hpp"

/**
 * The class of objects acting as a middle-man of communication with ESP-01S module.
 */
class Esp01SMessagingSurrogate
{
    static const uint16_t command_completed_event_id = 0x5000;

    codal::MessageBus event_bus;
    const std::unique_ptr<codal::MicroBitSerial> serial;
    std::queue<TimeStampedCommand> command_queue;

    void flushRx();

    void executeAndWaitCommand(Esp01SCustomCommand command);

    void sendCommand(Esp01SCustomCommand command);

    [[nodiscard]] std::string waitAndReadAsync(unsigned int size);

    /**
     * Expect to read the string immediately after 100ms has passed, and then clear RX.
     * @return true iff we have found the expected string on RX after sleeping for 100ms.
     */
    bool expectToRead(const std::string& expected);

public:
    Esp01SMessagingSurrogate(const MessageBus& message_bus,
                             NRF52Pin tx,
                             NRF52Pin rx,
                             uint8_t rxBufferSize = CODAL_SERIAL_DEFAULT_BUFFER_SIZE,
                             uint8_t txBufferSize = CODAL_SERIAL_DEFAULT_BUFFER_SIZE);;

    /**
     * Wait until ESP-01S module is able to answer us.
     */
    void init();

    void onSoundDetectedEvent();

    /**
     * Run the command-loop. The behaviour is undefined if this function is executed more than once.
     */
    [[noreturn]] void beginCommandLoop();

    /**
     * Run the task to ping remote every half a second.
     */
    [[noreturn]] void beginPingRemoteLoop();
};

[[noreturn]] void beginSurrogateCommandLoop(void* surrogate);

[[noreturn]] void beginPingRemoteLoop(void* surrogate);

