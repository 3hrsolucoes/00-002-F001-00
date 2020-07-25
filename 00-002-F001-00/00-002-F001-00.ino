#define vel_line 3
#define vel_curve_int -4
#define vel_curve_ext 10

#define motor_a_f 5
#define motor_a_r 9
#define motor_b_f 6
#define motor_b_r 10

#define sensor_1_ext 4
#define sensor_1_int 7
#define sensor_2_int 11
#define sensor_2_ext 12

#define sensor_v_1 3
#define sensor_v_2 2

#define trigger_pin 14
#define echo_pin 15

bool trigger=false;
bool echo_signal=false;
long time1=0;
float cm=0;
bool echo_read=false;
long timeout=0;

int pwm_1=0;
int pwm_2=0;

float Kp = 3;
float Ki = 0.6;
float Kd = 0.002;
float errord, errorLastd, erroInted, outputd;
float errore, errorLaste, erroIntee, outpute;

unsigned long encoder1 = 0;
unsigned long encoder2 = 0;
float pv_speed1 = 0;
float pv_speed2 = 0;
int pwm_pulse1_int=0;
int pwm_pulse2_int=0;
float set_speed_1 = 0;
float set_speed_2 = 0;
float pwm_pulse1 = 0;     //this value is 0~255
float pwm_pulse2 = 0;     //this value is 0~255

long t_delay=0;

void setup() {
  Serial.begin(9600);
  pinMode(motor_a_f,OUTPUT);
  pinMode(motor_a_r,OUTPUT);
  pinMode(motor_b_f,OUTPUT);
  pinMode(motor_b_r,OUTPUT);
  pinMode(sensor_1_ext,INPUT);
  pinMode(sensor_1_int,INPUT);
  pinMode(sensor_2_int,INPUT);
  pinMode(sensor_2_ext,INPUT);
  pinMode(trigger_pin,OUTPUT);
  pinMode(echo_pin,INPUT);
  digitalWrite(trigger_pin,LOW);
  pinMode(sensor_v_1,INPUT_PULLUP);
  pinMode(sensor_v_2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensor_v_1),detect_a,CHANGE);
  attachInterrupt(digitalPinToInterrupt(sensor_v_2),detect_b,CHANGE);
  interrupts();
}

void loop() {

  if(micros()-t_delay>=10000){
    read_distance();
    if (cm==0){
      cm=30;
    }
    Serial.println(cm);
    t_delay=micros();

      if (digitalRead(sensor_1_int) && digitalRead(sensor_2_int) && cm>15){
        set_speed_1=vel_line;
        set_speed_2=vel_line;
      }
      else if (!digitalRead(sensor_1_int) && !digitalRead(sensor_2_int)&& cm>15){
        set_speed_1=vel_line;
        set_speed_2=vel_line;
      }
      else if (!digitalRead(sensor_1_int) && digitalRead(sensor_2_int)&& cm>15){
        set_speed_1=vel_curve_ext;
        set_speed_2=vel_curve_int;
        Serial.println("e");
      }
      else if (digitalRead(sensor_1_int) && !digitalRead(sensor_2_int)&& cm>15){
        set_speed_1=vel_curve_int;
        set_speed_2=vel_curve_ext;
        Serial.println("d");
      }     
      else{
        motor_off();
      }   
    update_motors();
  }
}

void tik(){

    float inputd=set_speed_1;
    pv_speed1 = (600.0*(float(encoder1)/140.0));
    errord=inputd-pv_speed1;
    float errorDiffd;
    float outputd;
    errord = errord * 0.7 + inputd * 0.3; // filter
    errorDiffd = errord - errorLastd;
    erroInted = constrain(erroInted + errord, -120, 120);
    outputd = Kp * errord + Ki * erroInted + Kd * errorDiffd;
    errorLastd = errord;
    outputd=constrain(outputd,0,255);
    encoder1=0;
    pwm_pulse1=outputd;
        
    float inpute=set_speed_2;
    pv_speed2 = (600.0*(float(encoder2)/140.0));
    errore=inpute-pv_speed2;
    float errorDiffe;
    float outpute;
    errore = errore * 0.7 + inpute * 0.3; // filter
    errorDiffe = errore - errorLaste;
    erroIntee = constrain(erroIntee + errore, -120, 120);
    outpute = Kp * errore + Ki * erroIntee + Kd * errorDiffe;
    errorLaste = errore;
    outpute=constrain(outpute,0,255);
    encoder2=0;
    pwm_pulse2=outpute;

}

void detect_a(){
  encoder1+=1;
  //Serial.println("DETECT_A");
}
void detect_b(){
  encoder2+=1;
//  Serial.println("DETECT_B");
}
void read_distance(){
    digitalWrite(trigger_pin,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_pin,LOW);
    time1=pulseIn(echo_pin, HIGH,9000);
    cm=(time1)/58;
}
void motor_off(){
    set_speed_1=0;
    set_speed_2=0;
    tik();
    analogWrite(motor_a_f,0);
    analogWrite(motor_b_f,0);
}
void update_motors(){
    tik();
    pwm_pulse1_int=int(pwm_pulse1);
    pwm_pulse2_int=int(pwm_pulse2);
    analogWrite(motor_a_f,pwm_pulse1_int);
    analogWrite(motor_b_f,pwm_pulse2_int);
}
