/*!
   @file firmware_modules.h
   @brief This file determines which modules will be included in the slave firmware.
   
   Add other modules or comment out the existing ones as needed.
   @section author Author
   Copyright (c) 2022 juh
   @section license License
   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation, version 2.
*/



#include "AccelStepperI2C_firmware.h"
#include "ServoI2C_firmware.h"
#include "PinI2C_firmware.h"
#include "ESP32sensorsI2C_firmware.h"
