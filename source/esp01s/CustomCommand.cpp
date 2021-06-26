#include "CustomCommand.hpp"

using namespace esp_01s;

static uint16_t command_timestamp = 0;

TimeStampedCommand::TimeStampedCommand(CustomCommand command) :
        command(command),
        timestamp(command_timestamp) {}
