
#include "mpu.h"
#include "I2Cdev.h"
#include"RF24.h"
#include "freeram.h"

float pid_p_gain_roll = 1.4;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05;              //Gain setting for the roll I-controller (0.05)
float pid_d_gain_roll = 15;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)



byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36];
byte highByte, lowByte;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4, loop_counter;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int cal_int, start, gyro_address;
int receiver_input[5];
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer=0.0;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis[4], gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
unsigned long lastRecvTime = 0;


RF24 rx(8,9);



int ret;
void imu_setup() {
    Fastwire::setup(400,0);
    
     mympu_open(200);
    // delay(2000);
     freeRam();
    //Serial.print("MPU init: "); Serial.println(ret);
  
	
}


void motor_initialize(){
          PORTD &= B00001111;
          PORTD |= B11110000;
          delayMicroseconds(1000);
          PORTD &= B00001111; 
          delay(2500); 
          PORTD |= B11110000;
          delayMicroseconds(2000);
          PORTD &= B00001111;  
}



void setup(){
  Serial.begin(57600);
  motor_initialize();
 // Wire.begin();     //Start the I2C as master.
  imu_setup();
  rx_Initialize();
  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B11110000;                                           //Configure digital poort 3, 4, 5 and 6 as output.
  //DDRB |= B00110000;                                           //Configure digital poort 12 and 13 as output.
    
  
  
  
  for (cal_int = 0; cal_int < 1250 ; cal_int ++){              //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                        //Set digital poort 3, 4, 5 and 6 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 3, 4, 5 and 6 low.
    delayMicroseconds(3000);      //Wait 3000us.
    receiver_update();
    mympu_update();  
  }

      

  //Wait until the receiver is active and the throtle is set to the lower position.
  while(receiver_input_channel_3 < 990 || receiver_input_channel_3 > 1020 || receiver_input_channel_4 < 1400){
    receiver_update();
    start ++;                                                  //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                                        //Set digital poort 3, 4, 5 and 6 high.
    delayMicroseconds(1000);                                   //Wait 1000us.
    PORTD &= B00001111;                                        //Set digital poort 3, 4, 5 and 6 low.
    delay(3);    //Wait 3 milliseconds before the next loop.
    Serial.println(start);
    if(start == 125){                                          //Every 125 loops (500ms).
      //digitalWrite(12, !digitalRead(12));                      //Change the led status.
      start = 0;                                               //Start again at 0.
    }
  }
  start = 0;                                                   //Set start back to 0.
  
  battery_voltage = 1260;//(analogRead(0) + 65) * 1.2317;
  
}
void loop(){
  
  receiver_update();
  unsigned long now = millis();
  if(now - lastRecvTime > 1000)
  
  {
    signal_loss();
  }
  
  mympu_update();
  
  
  gyro_pitch = -(mympu.gyro[1]);
  gyro_roll = -(mympu.gyro[2]);
  gyro_yaw = -(mympu.gyro[0]);
  
  gyro_roll_input = (gyro_roll_input * 0.9) + ((gyro_roll) * 0.1);            //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.9) + ((gyro_pitch) * 0.1);         //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.9) + ((gyro_yaw) * 0.1);               //Gyro pid input is deg/sec.

  Serial.print(gyro_roll_input);
  Serial.print("\t");
  Serial.print(gyro_pitch_input);
  Serial.print("\t");
  Serial.println(gyro_yaw_input);
  


  //For starting the motors: throttle low and yaw left (step 1).
  if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
  Serial.println(start);
  if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450){
    start = 2;
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  if(start == 2 && receiver_input_channel_3 < 1175 && receiver_input_channel_4 > 1850)start = 0;
  
  pid_roll_setpoint = 0;
  if(receiver_input_channel_1 > 1504)pid_roll_setpoint = ((receiver_input_channel_1 - 1504)/24.0);
  else if(receiver_input_channel_1 < 1488)pid_roll_setpoint = ((receiver_input_channel_1 - 1488)/24.0);
  
  pid_pitch_setpoint = 0;
  if(receiver_input_channel_2 > 1504)pid_pitch_setpoint = -((receiver_input_channel_2 - 1504)/24.0);
  else if(receiver_input_channel_2 < 1488)pid_pitch_setpoint = -((receiver_input_channel_2 - 1488)/24.0);
  
  pid_yaw_setpoint = 0;
  if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1518)pid_yaw_setpoint = ((receiver_input_channel_4 - 1518)/24.0);
    else if(receiver_input_channel_4 < 1502)pid_yaw_setpoint = ((receiver_input_channel_4 - 1502)/24.0);
  }
  
  Serial.println(pid_roll_setpoint);
  Serial.println(pid_pitch_setpoint);
  Serial.println(pid_yaw_setpoint);


  calculate_pid();
  battery_voltage = 1260;//battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  
  //Turn on the led if battery voltage is to low.
  //if(battery_voltage < 1030 && battery_voltage > 600)//digitalWrite(12, HIGH);
  
  throttle = receiver_input_channel_3;                                      //We need the throttle signal as a base signal.
  
  if (start == 2){                                                          //The motors are started.
    if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 
    
    if (esc_1 < 1075) esc_1 = 1075;                                         //Keep the motors running.
    if (esc_2 < 1075) esc_2 = 1075;                                         //Keep the motors running.
    if (esc_3 < 1075) esc_3 = 1075;                                         //Keep the motors running.
    if (esc_4 < 1075) esc_4 = 1075;                                         //Keep the motors running.
    
    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }
  
  else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }
  
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 3,4,5 and 6 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 3 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 6 to low if the time is expired.
  }
  
  
}





void calculate_pid(){
  //Roll calculations
  //Serial.println("Roll");
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  
  //Pitch calculations
  //Serial.println("pitch");
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations
  //Serial.println("yaw");
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}


