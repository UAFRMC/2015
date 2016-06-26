//Author: Aven Bross
//Date: 3/30/2014
//Handles serial connection between backend and arduino

#ifndef __AURORA_ROBOTICS__ROBOT_SERIAL_H
#define __AURORA_ROBOTICS__ROBOT_SERIAL_H

#include "../serial.cpp" // hacky way, throwing undeclared for all serial functions if serial.h is included
#include <sstream>
// #include "robot.h"
#include "display.h"
#include "../cyberalaska/serial_packet.h"

class robot_serial {
public:
	A_packet_formatter<SerialPort> pkt;
	int _timeout;
	void connect();
	void update(robot_current &robot);
	uint32_t Mcountdiff, DLdiff, DRdiff; // Delta for mining head encoder

	robot_serial() :pkt(Serial) {
		_timeout=100; //< hack, to get connect at startup
		Mcountdiff = DLdiff = DRdiff = 0;
	}
};

// Attempt to connect to the arduino
void robot_serial::connect(){
	static int reset_count=0;
	if ((Serial.begin(9600) == -1)) // serial port to Arduino
	{
		if ((++reset_count%200)==0) { // try a resetusb
			int err=system("resetusb");
			robotPrintln("Resetusb return code: %d\n",err);
		}
		robotPrintln("Attempting to connect");
		usleep(100*1000); //100ms delay
	}
	else {
		robotPrintln("Opened Arduino port.  Waiting for data.");
		int r=0;
		while (-1==(r=Serial.read())) {}
		robotPrintln("Connected to Arduino.  First byte: %02x",r);
	}
}

void robot_serial::update(robot_current &robot){
	bool got_data=false;

	// Send off power command:
	if (_timeout==0) {
		pkt.write_packet(0x7,sizeof(robot.power),&robot.power);
	}

	// See if robot sends anything back:
	do {
		A_packet p;
		while (-1==pkt.read_packet(p)) { // receive packet data
			got_data=true;

		}
		if (p.valid)
		{
			if (p.command==0)
			{
				robotPrintln("Got echo packet back from robot");
			}
			else if (p.command==0xE)
			{
				robotPrintln("Got ERROR (0xE) packet back from robot (length %d)", p.length);
			}
			else if (p.command==0x3)
			{
				// sensor data
				if (!p.get(robot.sensor))
				{
					robotPrintln("Size mismatch on arduino -> PC sensor packet (expected %d, got %d)",sizeof(robot.sensor),p.length);
				}
				else
				{
					if(robot.power.mineEncoderReset!=0)
					{
						Mcountdiff=-robot.sensor.Mcount+120;
						Mcountdiff=Mcountdiff%120;
					}

					// got valid sensor report: arduino is OK
					robot.status.arduino=1;
					robot.sensor.Mcount += Mcountdiff;
					robot.sensor.Mcount = robot.sensor.Mcount % 120;
					robot.sensor.DLcount += DLdiff;
					robot.sensor.DRcount += DRdiff;
				}
			}
			else if (p.command==0xB)
			{ // blinky data
				robot_blinky_report blinky;
				if (!p.get(blinky))
				{
					robotPrintln("Size mismatch on arduino -> PC blinky packet (expected %d, got %d)",sizeof(blinky),p.length);
				}
				else
				{
				// Check timing
					static int last_time=0;
					int cur_time=blinky.timing;
					int expect_time=(last_time+robot_blinky_report::n_report)&3;
					if (expect_time!=cur_time)
						robotPrintln("  Blinky timing glitch of %d ms\n",
							cur_time-expect_time);
					last_time=cur_time;

				/*
				// Print packet
					robotPrintln("Got blinky packet back from robot:");
					for (int i=0;i<robot_blinky_report::n_report;i++)
					{
						robotPrintln("	analog %d",blinky.read(i));
						// should send data off to analysis now...
					}
				*/
				}
			}
			else
			{ // unknown packet type?!
				robotPrintln("Got unknown packet type 0x%x length %d from robot",p.command,p.length);
			}
		}
		else
		{ /* some sort of serial error? */
			break;
		}
	} while (Serial.available()); // keep reading to clean out buffer

	if(got_data)
	{
		_timeout=0;
	}
	else if(_timeout>5)
	{
		robot.status.arduino=0;
		robotPrintln("Connection Lost");
		Mcountdiff = robot.sensor.Mcount;
		Mcountdiff = Mcountdiff % 120;
		DLdiff = robot.sensor.DLcount;
		DRdiff = robot.sensor.DRcount;
		connect();
	}
	else{
		_timeout++;
	}

}

#endif
