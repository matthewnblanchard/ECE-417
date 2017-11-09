#include "/usr/local/include/labvolt/inc/labvolt.h"
#include <unistd.h>

// Joint angle --> motor step macros
#define BASE(X) 6500/360*X
#define SHOULDER(X) 8600/360*X
#define ELBOW(X) 8600/360*X
#define M4(X) 6500/360*X
#define M5(X) 6500/360*X

int main(int argc, char * argv[])
{
	init();
		
	zero();                      				// Remember the home position
	moverel(BASE(30),0,0,0,0);   				// Rotate base 30 degrees CCW
	moverel(0,SHOULDER(-20),ELBOW(20),M4(20),M5(-20)); 	// Shoulder down 20 degrees
	moverel(0,0,ELBOW(15),M4(20),M5(-20));			// Elbow down 15 degrees
	moverel(0,0,0,M4(-20),M5(-20));				// Pitch wrist 20 degrees up
	moverel(0,0,0,M4(-20),M5(20));				// Roll wrist 20 degrees CW
	sleep(5);						// Sleep for 5 seconds
	gripperClose();              				// Close the gripper
	nest();                     				// Return arm to home position
							
	shutdown();	
	return 0;	
}
