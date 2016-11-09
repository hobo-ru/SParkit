/*
 * commands.h
 *
 *  Created on: 14 июля 2014 г.
 *      Author: vdubikhin
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

enum SlaveMasterCommands {
    DataReceive = 1,
    Boot = 2,
    PowerOnLed = 3,
    ReleaseBus = 4,
    KeepSlaveAlive = 5,
    EEPROMStore = 6,
	startBootloader = 7
};

#endif /* COMMANDS_H_ */
