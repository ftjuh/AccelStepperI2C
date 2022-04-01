/*!
 *  @file template_I2C_firmware.h
 *  @brief Use this file for your own optional extension module of the I2Cwrapper
 *  firmware. Will usually be bundled with a matching master library. Needs to
 *  be activated in the firmware_modules.h configuration file.
 * 
 *  Currently, code can be injected at the following stages:
 * #define MF_STAGE_includes       1
 * #define MF_STAGE_declarations   2
 * #define MF_STAGE_setup          3
 * #define MF_STAGE_loop           4
 * #define MF_STAGE_processMessage 5
 * #define MF_STAGE_reset          6
 * 
 *  See below instructions on what code to place where. See existing modules
 *  for illustration, PinI2C_firmware.h and ServoI2C_firmware.h are good starting
 *  points.
 *  @section author Author
 *  Copyright (c) 2022 juh
 *  @section license License
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation, version 2.
 */

/*
 * Place include calls needed by your module here. Usually this will be the 
 * definition file of your master's library which defines the command codes used 
 * below and other relevant stuff.
 */
#if MF_STAGE == MF_STAGE_includes
#include <xxxI2C.h>
#endif // MF_STAGE_includes


/*
 * This code will be injected after the include section and before the setup 
 * function. Use it to declare variables, constants and supporting functions 
 * needed by your modules
 */
#if MF_STAGE == MF_STAGE_declarations

#endif // MF_STAGE_declarations


/*
 * This code will be injected into the slave's setup() function.
 */
#if MF_STAGE == MF_STAGE_setup
  log("###template### support enabled.\n");
  #endif // MF_STAGE_setup


/*
  * This code will be injected into the slave's loop() function.
  * You may use triggerInterrupt() here for, well, triggering interrupts.
  * Use uint32_t cycles if you need to know 
  */
#if MF_STAGE == MF_STAGE_loop
#endif // MF_STAGE_loop


/*
 * This code will be injected into the processMessage() function's main switch{}
 * statement. It is responsible for reading the master's commmands, processing 
 * them and, optionally, prepare some data to be sent back upon the master's next
 * request event.
 * 
 * The code consists of 0 to n instances of "case xxxCmd: {}" clauses which can 
 * use the following variables:
 * 
 * uint8:t xxxCmd -> command id, must be unique for that slave. Currently there
 * is no mechanism in place for controlling that, so it's up to you to ensure 
 * uniqeness of command iIDs. Usually, these are constants defined in an include
 * in the MF_STAGE_includes stage.
 * 
 * uint8_t unit -> the "unit" this command is adressed to (optional), e.g. which
 * of several stepper motors attached to the slave
 * 
 * uint8_t i -> the number of paramter bytes. Check this against the expected 
 * number of parameter bytes for each command
 * 
 * 
 * SimpleBuffer bufferIn()
 * 
 * SimpleBuffer bufferOut()
 */
#if MF_STAGE == MF_STAGE_processMessage

      case /* ###Cmd */: {
          if (i == /* parameter bytes this commands needs */) { 
            
            /* use this to read the defined parameters for this command from 
             * the input buffer */
            uint8_t test /* can be any data type */; 
            bufferIn->read(test); /* will read sizeof(test) bytes from the buffer*/
            
            /* now the slave should probably do sth meaningful with "test" */
            
            /* use this to write a commands' "reply" to the output buffer which 
             * will be sent upon the master's next request event */
            bufferOut->write(++test);
            
          } 
          /* If you want your slave to show some defined behavior if a command
           * does not come with the proper amount of parameter bytes, you could
           * do it here in an else() clause.
           */
        }
        break;

#endif // MF_STAGE_processMessage

        
/*
 * This code will be called if the slave device is to perform a hardware reset 
 * via I2C command. Use it to do any necessary cleanup, and to put the device 
 * in a safe initial state.
 */
#if MF_STAGE == MF_STAGE_reset
#endif // MF_STAGE_reset
        
