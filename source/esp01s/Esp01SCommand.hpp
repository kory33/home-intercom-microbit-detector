#pragma once

#include <cstdint>

enum class Esp01SCustomCommand : char {
    ConfirmStartup = 'C',
    PingRemote = 'R',
    NotifySoundDetection = 'N',
    QueryPreviousResult = 'P',
    ForgetPreviousResult = 'Q',
};

class TimeStampedCommand {
public:
    const Esp01SCustomCommand command;
    const uint16_t timestamp;

    explicit TimeStampedCommand(Esp01SCustomCommand command);
};
