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

float velocity_time=10;       //10ms after it calculate the velocity

              //PID Parameters
float kp=7,ki=2,kd=0.5;                  //tuning parameter
float error=0,sum_error=0,last_error=0; 
float pid=0;                            //pid computation result
int set_point=25;                       //desired velocity in terms of cm/s
int dt=velocity_time;

float change_distance=0;                      //do not alter this line
float new_time=0, old_time=0, change_time=1;  //do not alter this line
float velocity=0;

void setup() 
{
  Serial.begin(115200);
  pinmode_setup();
  myEnc.write(0);
}

void loop() 
{
  velocity=get_velocity_pid();
  motor_dir1();
  analogWrite(pwm1,pid);
  Serial.println(velocity);
}

float get_distance(void)     // FUNCTION for calculating the Distance after last time...
{
  float ch_distance;           
  ch_distance=myEnc.read();     //reading encoder
  myEnc.write(0);               //seting encoder to zero again
                  
  ch_distance/=1500;            // 1500 encoder values is equal to 1 revolution        
  ch_distance*=circumference;   //converting revolution to cm distance
  return ch_distance;
}

                              //FUNCTION for calculating the velocity and PID result after certain time. eg 10ms
float get_velocity_pid(void)
{
  new_time=millis();
  if((new_time-old_time)>=velocity_time)
  {
                              
  change_distance=get_distance(); //get distance
  velocity=change_distance*1000;  //1000 for miliseconds to seconds
  velocity/=(new_time-old_time);  //devide by time is velocity
  old_time=new_time;
  
  //GETTING compute its PID PID PID
  error=set_point-velocity;   //calculating Error
  sum_error+=error;           //summation of error
  pid=kp*error+ki*sum_error+kd*(error-last_error)/dt;     //PID result value(OUTPUT) 
  last_error=error;
  pid=constrain(pid,0,250);         // defines limit
  }
  return velocity;  
}

void pinmode_setup(void)
{
  pinMode(ch1,OUTPUT);
  pinMode(ch2,OUTPUT);
  pinMode(pwm1,OUTPUT);
}

void motor_dir1(void)
{
  digitalWrite(ch1,HIGH);
  digitalWrite(ch2,LOW);
}

void motor_dir2(void)
{
  digitalWrite(ch2,HIGH);
  digitalWrite(ch1,LOW);
}
