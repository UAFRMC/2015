/**
 * Aurora Robotics "Autonomy" firmware - 2015 BTS Version
 * For Arduino Mega
 */
#include "robot.h" /* classes shared with PC, and across network */
#include "serial_packet.h" /* CYBER-Alaska packetized serial comms */
#include "encoder.h"
#include "milli.h"
#include "pid.h"
#include "bts_motor.h"
#include "speed_controller.h"

/* Subtle: I don't need a namespace, but this at least disables the Arduino IDE's
utterly broken prototype generation code. */
namespace aurora {

// Hardware pin wiring (for Mega)
BTS_motor_t motor_M(3,4,200);
BTS_motor_t motor_L(5,6,60);
BTS_motor_t motor_R(7,8,60);
BTS_motor_t motor_F(9,10,255);

// All PC commands go via this (onboard USB) port
HardwareSerial &PCport=Serial; // direct PC
HardwareSerial &PCport2=Serial2; //donor's RX/TX2

// Call this function frequently--it's for minimum-latency operations
void low_latency_ops();

/** This class manages communication via an A_packet_formatter,
 including timeouts. */
class CommunicationChannel {
public:
  HardwareSerial &backend;
  A_packet_formatter<HardwareSerial> pkt; // packet formatter
  bool is_connected; // 1 if we're recently connected; 0 if no response
  milli_t last_read; // millis() the last time we got data back
  milli_t next_send; // millis() the next time we should send off data

  CommunicationChannel(HardwareSerial &new_backend) :
  backend(new_backend), pkt(backend)
  {
    is_connected=0;
    last_read=milli;
    next_send=milli;
  }

  bool read_packet(A_packet &p) {
    p.valid=0;
    if (backend.available()) {
      while (-1==pkt.read_packet(p)) {
        low_latency_ops(); /* while reading packet */
      }
      if (p.valid && p.length>0) {
        last_read=milli;
        next_send=milli;
        is_connected=true; // got valid packet
        digitalWrite(13,HIGH); // !!(milli&(1<<8))); // blink while getting good data
        return true;
      }
    }
    if (milli-next_send>500) { // read timeout
      next_send=milli;
      pkt.reset();
      pkt.write_packet(0,0,0); // send heartbeat ping packet
      is_connected=false;

      digitalWrite(13,0); // LED off if disconnected
    }
    return false;
  }
};
CommunicationChannel PC(PCport);
CommunicationChannel PCdonor(PCport2);




//encoder_t encoder_M(A3); // Mining head left side motor
speed_controller_t<2> encoder_M(1,0,0,A3,80,motor_M); // Mining head left side motor
//encoder_t encoder_DL(A4);  //Left wheel encoder
//encoder_t encoder_DR(A0); //Right wheel encoder
speed_controller_t<2> encoder_DL(1,0,0,A4,10,motor_L);  //Left wheel encoder
speed_controller_t<2> encoder_DR(1,0,0,A0,10,motor_R); //Right wheel encoder





/***************** Robot Control Logic ****************/
int pin_bumper_L=44;
int pin_bumper_R=47;
int pin_voltage_supply=49;
int front_encoder_power=52;

long mine_last=0; // millis() at last mining motion check
int bucket_last=0; // bucketFill at last mining motion check
int bucket_last2=0; // last-last

long stall_last=0; // timer for stall detection average
int sum_stalled=0, sum_stall_count=0;

int bts_enable_pin=11;


// Robot's current state:
robot_current robot;

// Read all robot sensors into robot.sensor
void read_sensors(void) {
  /*
  2015 control board:
   */
  robot.sensor.battery=analogRead(A12); // 24V bus
  low_latency_ops();
  robot.sensor.bucket=analogRead(A1); // bucket linear sensor
  low_latency_ops();
  robot.sensor.Mspeed=analogRead(A2);

  robot.sensor.stop=robot.sensor.battery<600;
  
  robot.sensor.Mstall=encoder_M.stalled;
  robot.sensor.DRstall=encoder_DL.stalled;
  robot.sensor.DLstall=encoder_DR.stalled;
}

// Match up motor power value with encoder
void send_motor_power(int power64,BTS_motor_t &motor,encoder_t &enc) {
	// update encoder direction
	if (power64>64) enc.last_dir=+1;
	if (power64<64) enc.last_dir=-1;
	// send to motor
	motor.drive(power64);
}

// Send current power values to the motors
void send_motors(void)
{
  int drivePower=100;
  if(robot.power.high)
  {
    drivePower=255;
  }
  motor_L.max_power=drivePower;
  motor_R.max_power=drivePower;
  
  if(robot.power.motorControllerReset!=0)
    digitalWrite(bts_enable_pin,LOW);
  else
    digitalWrite(bts_enable_pin,HIGH);

  int left=encoder_DL.update(robot.power.left,robot.power.torqueControl==0);
  send_motor_power(left,motor_L,encoder_DL);

  int right=encoder_DR.update(robot.power.right,robot.power.torqueControl==0);
  send_motor_power(right,motor_R,encoder_DR);

  // automated mining at fixed rate
  int mine=robot.power.mine;
  mine=encoder_M.update(mine,robot.power.mineMode!=0||robot.power.mineDump!=0);
  send_motor_power(mine,motor_M,encoder_M);

  motor_F.drive(robot.power.dump);
}

// Structured communication with PC:
void handle_packet(A_packet_formatter<HardwareSerial> &pkt,const A_packet &p)
{
  if (p.command==0x7) { // motor power commands
    low_latency_ops();
    if (!p.get(robot.power)) { // error
      pkt.write_packet(0xE,0,0);
    }
    else
    { // got power request successfully: read and send sensors
      low_latency_ops(); /* while reading sensors */
      read_sensors();
      low_latency_ops();
      pkt.write_packet(0x3,sizeof(robot.sensor),&robot.sensor);
      robot.sensor.latency=0; // reset latency metric
      low_latency_ops();
    }
  }
  else if (p.command==0) { // ping request
    pkt.write_packet(0,p.length,p.data); // ping reply
  }
}



/**** Low latency (sub-millisecond) timing section.
 * We need this for maximum accuracy in our encoder counts.
 */
robot_blinky_report blinky_report;
unsigned int blinky_index=0;
unsigned int blinky_sync=0;

milli_t last_milli=0;
void low_latency_ops() {
  unsigned long micro=micros();
  milli=micro>>10; // approximately == milliseconds

  //Encoder for mining motor
  encoder_M.read();
  // robot.sensor.Mspeed=encoder_M.period;
  robot.sensor.Mcount=encoder_M.count_120;


  //Encoder stuff for left drive track
  encoder_DL.read();
  robot.sensor.DLcount=encoder_DL.count_dir;

  //Encoder stuff for rightt drive track
  encoder_DR.read();
  robot.sensor.DRcount=encoder_DR.count_dir;




#if 0 /* no need for blinky */

  if ((PC.is_connected || PCdonor.is_connected) && cycle>cycle_end)
  { /* wait for timer tickover, then take sample */
    while (((micro=micros())%1024u)>cycle_end) { /* busywait */
    }
    unsigned int blinky_value=analogRead(A7);
    micro=last_micro_loop=micros(); // ignore above for latency calculation

    digitalWrite(36, blinky_sync);// swap LED, for next reading
    blinky_sync=!blinky_sync;

    blinky_report.write(blinky_index,blinky_value);
    blinky_index++;
    if (blinky_index>=robot_blinky_report::n_report) {
      blinky_index=0;
      blinky_report.sync=blinky_sync;
      blinky_report.timing=micro/1024;
      PC.pkt.write_packet(0xB,sizeof(blinky_report),&blinky_report);
      PCdonor.pkt.write_packet(0xB,sizeof(blinky_report),&blinky_report);
    }
  }
#endif

  // Update latency counter
  unsigned int latency=milli-last_milli;
  if (latency>=1024) latency=1023;
  if (robot.sensor.latency<latency) robot.sensor.latency=latency;
  last_milli=milli;
}


}; // end namespace aurora
using namespace aurora;

void setup()
{
  Serial.begin(9600); // Control connection to PC via USB
  Serial2.begin(9600); // Control connection to PC via opto isolator

  // Our ONE debug LED!
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  // BTS Enable Pin (Controls all pins)
  pinMode(bts_enable_pin,OUTPUT);
  digitalWrite(bts_enable_pin,HIGH);

  // High supply for optical sensors
  pinMode(A5,OUTPUT);
  digitalWrite(A5,HIGH);
  pinMode(A11,OUTPUT);
  digitalWrite(A11,HIGH);

  // Pin modes for inputs
  //pinMode(pin_bumper_L,INPUT_PULLUP);
  //pinMode(pin_bumper_R,INPUT_PULLUP);
  pinMode(pin_voltage_supply,OUTPUT);
  pinMode(front_encoder_power,OUTPUT);
  digitalWrite(pin_voltage_supply,HIGH);
  digitalWrite(front_encoder_power,HIGH);
}


milli_t milli;
milli_t next_milli_send=0;
void loop()
{
  low_latency_ops();

  A_packet p;
  if (PC.read_packet(p)) handle_packet(PC.pkt,p);
  if (PCdonor.read_packet(p)) handle_packet(PCdonor.pkt,p);
  if (!(PC.is_connected||PCdonor.is_connected)) robot.power.stop(); // disconnected?

  if (milli-next_milli_send>=5)
  { // Send commands to motors
    send_motors();
    next_milli_send=milli; // send_motors every 5ms
  }
}

