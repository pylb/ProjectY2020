/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <unistd.h>
#include "csrv.h"

int main(int argc, char* argv[]) {
	if (csrvInit(NULL, NULL) != 0)
		return -1;

	// loop forever
	for (;;)
		sleep(10);
}
