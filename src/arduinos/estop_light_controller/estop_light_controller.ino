/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <Arduino.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

int blinkSigPin = 6; // connects to the second arduino.
int button1 = 2;
int button2 = 3;
int button3 = 4;
int button4 = 5;
int wirelessEstop = 7;

void blinkLight( const std_msgs::Int8& light_State){
  if (light_State.data == 2){
  digitalWrite(blinkSigPin, HIGH);   // signal other Arduino to blink the led
  }
  else {
    digitalWrite(blinkSigPin, LOW); //keep the arduino on while teleop or manual
  }
}

ros::Subscriber<std_msgs::Int8> sub("/gem/operation_mode", &blinkLight );
std_msgs::Bool bool_msg;
ros::Publisher Estop("gem/eStop", &bool_msg);

void setup()
{
  pinMode(blinkSigPin, OUTPUT);
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(button3, INPUT);
  pinMode(button4, INPUT);
  pinMode(wirelessEstop, INPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(Estop);
}

void loop()
{  
  bool_msg.data = buttonSurvey();
  nh.spinOnce();
  Estop.publish( &bool_msg);
  delay(100);
}

bool buttonSurvey(){
  if (digitalRead(wirelessEstop) || !digitalRead(button1)  || !digitalRead(button2) || !digitalRead(button3) || !digitalRead(button4)){
    return true;
  }
  else{
    return false;
  }
}
