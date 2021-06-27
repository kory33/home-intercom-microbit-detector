#pragma once

#include <cstdint>

namespace esp_01s {
    /**
     * Commands to send to esp-01s module.
     *
     * For ease of implementation, all of these commands are supposed to be idempotent:
     * on multiple successive executions of a single command, we expect
     *  - the side-effect from esp-01s to happen
     *  - to get the message DONE:{command sent} as many times as we have sent the command.
     */
    enum class CustomCommand : char {
        ConfirmStartup = 'C',
        PingRemote = 'P',
        NotifySoundDetection = 'N',

        // control commands
        GetReadyForNextCommand = 'G',
    };

    class TimeStampedCommand {
    public:
        const CustomCommand command;
        const uint16_t timestamp;

        explicit TimeStampedCommand(CustomCommand command);
    };
}
