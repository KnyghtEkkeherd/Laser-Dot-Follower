#ifndef _BLUETOOTH_
#define _BLUETOOTH_

#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

struct BlueetoothCommand{
    char command[7];
    int8_t servo;
    int8_t data;
};

struct BlueetoothCommand servoCommands[] = {
    // Commands for servos
    {"RESET0", 0, 0},

    {"Rbs045", 1, 45}, // Rotate bottom servo 45
    {"Rbs090", 1, 90},



    {"Rls045", 2, 45}, // Rotate lower servo 45
    {"Rls090", 2, 90},

    {"Rus045", 4, 45}, // Rotate upper servo 45
    {"Rus090", 4, 90},

    {"ClwGrb", 3, 0}, // Open claw
    {"ClwOpn", 3, 90}, // Release claw

    // Every servo at 0deg
    {"Rbs000", 1, 0},
    {"Rls000", 2, 0},
    {"Rup000", 4, 0},
    {"Clw000", 3, 0},

    // Every servo at 180deg
    {"Rbs180", 1, 90},
    {"Rls180", 2, 180},
    {"Rup180", 4, 180},
    {"Clw180", 3, 180},

	//instruction
	{"Rbsiii", 1, 0},
	{"Rbsooo", 1, 135},
	{"Rlslll", 2, 30},
	{"Rlshhh", 2, 110},
	{"Clwccc", 3, 0},
	{"Clwooo", 3, 90},
	{"Grab00",0,0}

};

#endif
