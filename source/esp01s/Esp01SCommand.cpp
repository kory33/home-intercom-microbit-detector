#include "Esp01SCommand.hpp"

static uint16_t command_timestamp = 0;

TimeStampedCommand::TimeStampedCommand(Esp01SCustomCommand command) :
        command(command),
        timestamp(command_timestamp) {}
