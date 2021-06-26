#pragma once

#include <cstdint>

namespace esp_01s {
    enum class CustomCommand : char {
        ConfirmStartup = 'C',
        PingRemote = 'R',
        NotifySoundDetection = 'N',
        QueryPreviousResult = 'P',
        ForgetPreviousResult = 'Q',
    };

    class TimeStampedCommand {
    public:
        const CustomCommand command;
        const uint16_t timestamp;

        explicit TimeStampedCommand(CustomCommand command);
    };
}
