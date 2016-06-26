/**
  Aurora Robotics keyboard user interface code.

  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#ifndef __AURORA_ROBOTICS__UI_H
#define __AURORA_ROBOTICS__UI_H

#include "ogl/event.h" /* for joystick */

/**
 Keyboard-based user interface for robot.
*/
class robot_ui {
public:
	robot_power power; // Last output power commands

	// Current floating-point power values:
	float left, right, front, mine, dump;

	// Human-readable description of current state
	std::string description;

	void stop(void) {
		left=right=front=mine=dump=0.0;
		power.stop();
		description="Sending STOP";
	}

	// Respond to these keystrokes.
	//  The "keys" array is indexed by the character, 0 for up, 1 for down.
	void update(int keys[],const robot_current &robot);

	robot_ui() {
		stop();
		description="Starting up";
	}

	// Clamp this float within this maximum range
	float limit(float v,float maxPower) const {
		if (v>maxPower) v=maxPower;
		if (v<-maxPower) v=-maxPower;
		return v;
	}

	// Convert a raw float to a motor command, with this maximum range
	byte toMotor(float v,float maxPower) const {
		v=limit(v,maxPower);
		int iv=(int)(v*63+64);
		if (iv<1) iv=1;
		if (iv>127) iv=127;
		return iv;
	}
};

void robot_ui::update(int keys[],const robot_current &robot) {
	static int keys_last[256];
	int keys_once[256]; // key down only *one* time per press
	for (int i=0;i<256;i++) {
		keys_once[i]= (keys[i] && !keys_last[i]);
		keys_last[i]=keys[i];
	}
	description="UI:\n";

// Power limits:
	float driveLimit=0.5;
	float mineLimit=1.0;
	float dumpLimit=1.0;

// Prepare a command:
	if (keys[' ']) { // spacebar--full stop
		stop();
		robotState_requested=state_STOP;
		return; // don't do anything else.  Just stop.
	}
	if(power.high)
	{
           description+="  ULTRA POWER\n";
           driveLimit=0.8;
	}
	// else spacebar not down--check other keys for manual control

/* scale back, so things die away when key is released.
   It'd be better to detect key-up here, and stop, but that's ncurses. */
        left*=0.80;
        right*=0.80;
        front*=0.5;
	mine*=0.5;
        dump*=0.5;



	static bool joyDrive=false;
	bool joyDone=false; // subtle:


/*  Fix: Uses only the left analog stick for driving the robot */
	/* Uses the left analog stick*/
/* TODO: Map dumpMode to joystick*/
	float forward=-oglAxis(2,"Go Forward or Reverse"); // left Y
	float turn=0.9*oglAxis(1,"Turn Left or Right"); //left X, scaled for gentle turns

	/* Oerate the mining head*/
	 float dumpJoy=-oglAxis(3,"Operate Dump Buckets");
	 float mineJoy=oglAxis(4,"Run Mine Head");

	if(forward!=0.0 || turn!=0.0 || dumpJoy!=0 || mineJoy!=0)
	{
		joyDrive=true; // using joystick
	}
	else {joyDone=true;}
	if(joyDrive)
	{
		left=driveLimit*(forward+turn);
		right=driveLimit*(forward-turn);
         	dump=dumpLimit*dumpJoy;
		mine+=mineJoy;

		description += "  joystick\n";
	}
	joyDrive=!joyDone;
	if(oglButton(1,"state_mine"))
	{
		robotState_requested=state_mine;
	}
	if(oglButton(2,"state_mine_hooks"))
	{
		robotState_requested=state_mine_hooks;
	}

	if (oglButton(3,"Stop"))
	{
		stop();
		robotState_requested=state_STOP;
	}
	if(oglButton(4,"Drive"))
	{
		robotState_requested=state_drive;
	}
	if(oglButton(5,"state_dump_pull"))
	{
		robotState_requested=state_dump_pull;
	}
	if((oglButtonOnce(6,"High Power")))
	{
		power.high=!power.high;
	}


	if ((oglButtonOnce(1,"Break"))) { stop(); } // stop without changing state


//Pilot warning messages:
	if(robot.sensor.bucket > head_mine_start||robot.sensor.bucket > head_bar_clear)
	{
		description += "  DO NOT MINE:\n";
	}
	if(robot.sensor.bucket > head_mine_start)
	{
		 description += "  Head high\n";
	}
	if(robot.sensor.bucket > head_bar_clear)
	{
		description += "  Hits Bar\n";
	}
	if(robot.sensor.bucket > head_drive_safe)
	{
		description += "  DO NOT DRIVE: Head high\n";
	}
	if(robot.sensor.bucket < head_mine_dump)
	{
		description += "  DO NOT DUMP: Head low\n";
		//description += '\n';
	}
	if(robot.sensor.Mstall)
		description+="  MINING HEAD STALLED\n";
	if(robot.sensor.DLstall)
		description+="  LEFT DRIVE STALLED\n";
	if(robot.sensor.DRstall)
		description+="  RIGHT DRIVE STALLED\n";

// Drive keys:
	float acceleration=.2;
        if(keys['w']||keys['W'])
        {
            left+=acceleration;
            right+=acceleration;
        }
        if(keys['s']||keys['S'])
        {
            left-=acceleration;
            right-=acceleration;
        }
        if(keys['a']||keys['A'])
        {
            left-=acceleration;
            right+=acceleration;
        }
        if(keys['d']||keys['D'])
        {
            left+=acceleration;
            right-=acceleration;
        }
        if(keys_once['t']||keys_once['T'])
        {
		power.torqueControl=!power.torqueControl;
        }

	power.motorControllerReset=keys['r']||keys['R'];

	if(power.motorControllerReset!=0)
		description+="  BTS Motor Reset\n";


        if(keys['0']) // zero out Mcount encoder (after lining up head)
        {
			power.mineEncoderReset=true;
        }
	else {
			power.mineEncoderReset=false;
	}

	if(power.torqueControl!=0)
		description+="  torque control\n";
	else
		description+="  speed control\n";


// Special features

	if(keys_once['m'])
        {
	   power.mineMode=!power.mineMode;
        }
	if (power.mineMode) {
		mine=0.5;
	}

	if(keys['j'])
        {

            mine=-1;
            description+="  mine-\n";
        }
	if(keys_once['p'])
	{
	    power.high=!power.high;
	}

        if(keys[oglSpecialDown])
        {
            dump=-1;
            description+="  dump-\n";
        }
        if(keys[oglSpecialUp])
        {
            dump=+1;
            description+="  dump+\n";
        }
        if(keys[oglSpecialRight])
        {
            mine=+0.2;
            description+="  mine+\n";
        }
        if(keys[oglSpecialLeft])
        {
            mine=-0.2;
            description+="  mine-\n";
        }


	left=limit(left,driveLimit);
	right=limit(right,driveLimit);


	power.left=toMotor(left,driveLimit);
	power.right=toMotor(right,driveLimit);
	power.mine=toMotor(mine,mineLimit);
	power.dump=toMotor(dump,dumpLimit);
}



#endif

