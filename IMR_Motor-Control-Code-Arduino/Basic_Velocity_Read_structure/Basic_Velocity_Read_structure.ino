#define ch1 13
#define ch2 12
#define pwm1 10


#include <Encoder.h>
Encoder myEnc(3, 5); 

//               DISTANCE MEASUREMENT UNIT
float distance_unit =1 ;    // 1=cm, 100=meters, 1000=km
//               WHEEL RADIUS
float wheel_radius=5/distance_unit;    //diameter in terms of "cm" (Put in cm always)

float circumference=wheel_radius*3.142*distance_unit;

float velocity_time=10;       //10ms

float change_distance=0;                      //do not alter this line
float new_time=0, old_time=0, change_time=1;  //do not alter this line
float velocity=0;

void setup() {
Serial.begin(115200);
pinmode_setup();
myEnc.write(0);
}

void loop() {

velocity=get_velocity();
motor_dir2();
analogWrite(pwm1,50);
//Serial.println(velocity);
}
void pinmode_setup(void)
{pinMode(ch1,OUTPUT);
pinMode(ch2,OUTPUT);
pinMode(pwm1,OUTPUT);}

void motor_dir1(void)
{digitalWrite(ch1,HIGH);
digitalWrite(ch2,LOW);}

void motor_dir2(void)
{digitalWrite(ch2,HIGH);
digitalWrite(ch1,LOW);}


float get_distance(void)
{float ch_distance;
ch_distance=myEnc.read();
myEnc.write(0);
ch_distance/=1500;
ch_distance*=circumference;   //converting revolution to cm
return ch_distance;}

float get_velocity(void)
{
new_time=millis();
if((new_time-old_time)>=velocity_time)
{
change_distance=get_distance();
velocity=change_distance*1000;
velocity/=(new_time-old_time);
Serial.println(velocity);
old_time=new_time;}
return velocity;  }
