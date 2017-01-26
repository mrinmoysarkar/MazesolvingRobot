/*
The programme is written by Mrinmoy sarkar.
 It is a project for iARC Robotic Competetion TECHKRITI 2014
 */
//all header file
#include<EEPROM.h>
#include<SD.h>
#include<LiquidCrystal.h>

// all variable
int stop_flag = 1;
String path = "";
String short_path = "";
//int speed_m = 100;
int dlay = 3000;
int forward_speed = 80;
int forward_speed_for_straight = 60;
int left_speed = 80;
int right_speed = 80;
int u_speed = 80;
int backward_speed = 100;
int l_delay = 200;
float speed_ratio_n = .8;
float speed_ratio_l = .7;
int Light_sensor_data_forward = 0;//to store the sensor value
int Light_sensor_data_backward = 0;
// to indicate left turn
int thres_val_mid = 110;
int mid_pin = 10;

double distance;//to store the distance of any object
double threshold_distance = 5;//distance from the destination box
//buzzer pin
int buzzer_pin = 21;
//all sensor input pin
int precision = 100;
int total_sensor = 10;//no of sensors
int white[10] = {
  110,110,110,110,110,110,110,110,110,110};//referance for white or black

//motor control pin
int right_motor_forward = 4;
int right_motor_backward = 5;
int left_motor_forward = 6;
int left_motor_backward = 7;
//sonor pin
int sonor_input_pin = 15;
int sonor_trigger_pin = 16;

int calibrate_switch = 0;

int temp_val=0;
int flag = 0;
int fl = 0;
int sdflag = 0;
boolean left_flag = false;
boolean right_flag = false;

void setup()
{
  pinMode(calibrate_switch,INPUT);
  pinMode(buzzer_pin,OUTPUT);

  pinMode(left_motor_forward,OUTPUT);
  pinMode(left_motor_backward,OUTPUT);
  pinMode(right_motor_forward,OUTPUT);
  pinMode(right_motor_backward,OUTPUT);

  pinMode(sonor_input_pin,INPUT);
  pinMode(sonor_trigger_pin,OUTPUT); 
  //initialize sd   card
  Serial.begin(9600);
  //init_sd();
  calibrate_sensor();
  
  delay(5000);
  check_sensor_data();
  blow_buzzer(200);
}

void loop()
{
  navigate_arena();
  //flow_line();
  //check_sensor_data();
  //right_turn();
  //short_path = "R";
  //go_through_shortest_path();
  while(1);
}


void flow_line()
{
  get_sensor_data();
  if((Light_sensor_data_forward == 0) && (Light_sensor_data_backward == 0))
  {
    stop_motor();
  }
  else if(!((Light_sensor_data_forward & 1 ) > 0 || (Light_sensor_data_forward & 16)>0))
  {
    get_sensor_data();
    if(((Light_sensor_data_forward & 0b1110) == 0b100) || ((Light_sensor_data_forward & 0b1110) == 0b1110))
    {
      analog_go_forward(forward_speed);
    }
    else if(((Light_sensor_data_forward & 0b1110) == 0b110) || ((Light_sensor_data_forward & 0b1110) == 0b010))
    {
      analog_go_left_n(left_speed,speed_ratio_l);
    }
    else if(((Light_sensor_data_forward & 0b1110) == 0b1100) || ((Light_sensor_data_forward & 0b1110) == 0b1000))
    {
      analog_go_right_n(right_speed,speed_ratio_l);
    }
  }
  else
  {
    get_sensor_data();
    if(((Light_sensor_data_backward & 0b1110) == 0b100) || ((Light_sensor_data_forward & 0b1110) == 0b1110))
    {
      analog_go_forward(backward_speed);
    }
    else if(((Light_sensor_data_backward & 0b1110) == 0b110) || ((Light_sensor_data_backward & 0b1110) == 0b010))
    {
      analog_go_left_n(left_speed,speed_ratio_l);
    }
    else if(((Light_sensor_data_backward & 0b1110) == 0b1100) || ((Light_sensor_data_backward & 0b1110) == 0b1000))
    {
      analog_go_right_n(right_speed,speed_ratio_l);
    }
  }
}





void navigate_arena()
{
  while(stop_flag)
  {
    get_sensor_data();
    if((Light_sensor_data_forward & 1) == 1)
    {
      left_flag = true;
    }
    else if((Light_sensor_data_forward & 0b10000) == 0b10000)
    {
      right_flag = true;
    }
    if(((Light_sensor_data_backward & 1) == 1) && left_flag)
    {
      stop_motor();
      left_turn();
      delay(70);
    }
    else if((!left_flag) && (Light_sensor_data_forward != 0) && ((Light_sensor_data_backward & 0b10000) == 0b10000))
    {
      stop_motor();
      straight();
      delay(70);
    }
    else if((!left_flag) && right_flag && (Light_sensor_data_forward == 0) && ((Light_sensor_data_backward & 0b10000)== 0b10000))
    {
      stop_motor();
      right_turn();
      delay(70);
    }
    else if((Light_sensor_data_forward == 0) && (Light_sensor_data_backward == 0) && (check_mid() == 0))
    {
      stop_motor();
      u_turn();
    }
    flow_line();
    end_point_check();
  }
}

void left_turn()
{
  left_flag = false;
  path += "L";
  while(stop_flag)
  {
    get_sensor_data();
    if((Light_sensor_data_forward & 0b1110) == 0)
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_left(u_speed);
    }
    end_point_check();
  }
  while(stop_flag)
  {
    get_sensor_data();
    int sensor_data = Light_sensor_data_forward & 0b11111;
    if((sensor_data == 0b00100) || (sensor_data == 0b01100) || (sensor_data == 0b00110))
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_left(u_speed);
    }
    end_point_check();
  }
}


void straight()
{
  path += "S";
  while(1)
  {
    flow_line();
    get_sensor_data();
    if(((Light_sensor_data_backward & 0b1) == 0) && ((Light_sensor_data_backward & 0b10000) == 0))
    {
      stop_motor();
      break;
    }
  }
}

void u_turn()
{
  path += "U";
  while(1)
  {
    get_sensor_data();
    if((Light_sensor_data_forward & 0b1110) == 0)
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_left(u_speed);
    }
  }
  while(1)
  {
    get_sensor_data();
    int sensor_data = Light_sensor_data_forward & 0b11111;
    if((sensor_data == 0b00100) || (sensor_data == 0b01100) || (sensor_data == 0b00110))
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_left(u_speed);
    }
  } 
}

void right_turn()
{
  right_flag = false;
  path += "R";
  while(1)
  {
    get_sensor_data();
    if((Light_sensor_data_forward & 0b1110) == 0)
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_right(u_speed);
    }
  }
  while(1)
  {
    get_sensor_data();
    int sensor_data = Light_sensor_data_forward & 0b11111;
    if((sensor_data == 0b00100) || (sensor_data == 0b01100) || (sensor_data == 0b00110))
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_right(u_speed);
    }
  }
}

void go_through_shortest_path()
{
  int indx = 0;
  int len = short_path.length();
  u_turn();
  delay(100);
  while(1)
  {
    get_sensor_data();
    if(((Light_sensor_data_backward & 1) == 1)||((Light_sensor_data_backward & 0b10000)== 0b10000))
    {
      if(short_path[indx] == 'R')
      {
        right_turn();
        indx++;
      }
      else if(short_path[indx] == 'L')
      {
        left_turn();
        indx++;
      }
      else if(short_path[indx] == 'S')
      {
        straight();
        indx++;
      }
    }
    if(indx == len)
    {
      stop_motor();
      break;
    }
    flow_line();
  }
  while(1)
  {
    flow_line();
    get_sensor_data();
    if((Light_sensor_data_forward == 0) || (Light_sensor_data_forward == 0b11111))
    {
      stop_motor();
      break;
    }
  }
}








void end_point_check()
{
  get_sensor_data();
  boolean t = ((Light_sensor_data_forward == 0b11111) && (check_mid()==1) &&  (((Light_sensor_data_backward & 0b1111) == 0b1111) || ((Light_sensor_data_backward & 0b11110) == 0b11110)));
  if(t)
  {
    stop_flag = 0;
    stop_motor();
  }
}



void do_the_job()
{
  navigate_arena();
  Serial.println(path);
  short_path = solve_maze(path);
  Serial.println(short_path);
  delay(500);
  go_through_shortest_path();
}

void calibrate_sensor()
{
  blow_buzzer(1000);
  int analog_data = 0;
  for(int i = 0; i < total_sensor; i++)
  {
    analog_data = analogRead(i);
    //Serial.println(analog_data);
    white[i] = int(analog_data + precision);
    delay(1);
  }
  analog_data = analogRead(mid_pin);
  thres_val_mid = analog_data + precision;
  delay(dlay);
  blow_buzzer(1000);
}

void blow_buzzer(int delay_time)
{
  digitalWrite(buzzer_pin,HIGH);
  delay(delay_time);
  digitalWrite(buzzer_pin,LOW);
}

void stop_motor()
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}

void analog_go_right_n(int speed_m,float speed_ratio)
{

  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  analogWrite(left_motor_forward,speed_m);
  analogWrite(right_motor_forward,int(speed_m * speed_ratio));
}

void analog_go_backright(int speed_m)
{
  analogWrite(left_motor_backward,speed_m);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}

void analog_go_u_right(int speed_m)
{
  digitalWrite(left_motor_backward,LOW);
  analogWrite(right_motor_backward,speed_m);
  analogWrite(left_motor_forward,speed_m);
  digitalWrite(right_motor_forward,LOW); 
}

void analog_go_left_n(int speed_m,float speed_ratio)
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  analogWrite(left_motor_forward,int(speed_ratio*speed_m));
  analogWrite(right_motor_forward,speed_m);
}

void analog_go_backleft(int speed_m)
{
  digitalWrite(left_motor_backward,LOW);
  analogWrite(right_motor_backward,speed_m);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}
void analog_go_u_left(int speed_m)
{
  analogWrite(left_motor_backward,speed_m);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  analogWrite(right_motor_forward,speed_m);
}

void analog_go_forward(int speed_m)
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  analogWrite(left_motor_forward,speed_m);
  analogWrite(right_motor_forward,speed_m);
}


void analog_go_backward(int speed_m)
{
  analogWrite(left_motor_backward,speed_m);
  analogWrite(right_motor_backward,speed_m);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}


void get_sensor_data()
{
  unsigned int Light_sensor_data = 0;
  int analog_data = 0;
  for(int i = 0; i < total_sensor; i++)
  {
    analog_data = analogRead(i);
    //Serial.println(analog_data);
    if(analog_data > white[i])
    {
      Light_sensor_data |= 1<<(i);
    }
    delay(1);
  }
  Light_sensor_data_forward = Light_sensor_data & 31;
  Light_sensor_data_backward = (Light_sensor_data & 512)>>9 | (Light_sensor_data & 224)>>4 | (Light_sensor_data & 256)>>4;
}


void get_sensor_data_ck()
{
  unsigned int Light_sensor_data = 0;
  int analog_data = 0;
  for(int i = 0; i < total_sensor; i++)
  {
    analog_data = analogRead(i);
    Serial.println(analog_data);
    if(analog_data > white[i])
    {
      Light_sensor_data |= 1<<(i);
    }
    delay(1);
  }
  Light_sensor_data_forward = Light_sensor_data & 31;
  Light_sensor_data_backward = (Light_sensor_data & 512)>>9 | (Light_sensor_data & 224)>>4 | (Light_sensor_data & 256)>>4;
}


int check_mid()
{
  int sensor_mid_val = analogRead(mid_pin);
  //  Serial.println(sensor_mid_val);
  if(sensor_mid_val < thres_val_mid)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

int check_mid_ck()
{
  int sensor_mid_val = analogRead(mid_pin);
  Serial.println(sensor_mid_val);
  if(sensor_mid_val < thres_val_mid)
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

double get_distance()
{
  digitalWrite(sonor_trigger_pin,HIGH);
  delayMicroseconds(10);
  digitalWrite(sonor_trigger_pin,LOW);
  int i = pulseIn(sonor_input_pin,HIGH);
  double distance = 0.017 * i;
  delay(3);
  return distance;
}

void check_sensor_data()
{
  get_sensor_data_ck();
  Serial.println("****************************");
  Serial.println("forward sensor");
  Serial.println(Light_sensor_data_forward,BIN);
  Serial.println("backward sensor");
  Serial.println(Light_sensor_data_backward,BIN);
  Serial.println("mid:");
  Serial.println(check_mid_ck());
  Serial.println("****************************");
  delay(3000);
}

void init_sd()
{
  pinMode(53,OUTPUT);
  if(!SD.begin(53))
  {
    Serial.println("Error opening sd card");
    sdflag = 1;
  }
  else
  {
    Serial.println("successful");
  }
}

void read_sd()
{
  if(sdflag == 1)
  {
    Serial.println("Error opening sd card in read");
  }
  else
  {
    File data_file;
    data_file = SD.open("path.txt");
    if(data_file)
    {
      while(data_file.available())
      {
        Serial.write(data_file.read());
      }
      data_file.close();
    }
  }
}

void write_sd(String data)
{
  if(sdflag == 1)
  {
    Serial.println("Error opening sd card in write");
  }
  else
  {
    File data_file;
    if(SD.remove("path.txt"))
    {
      data_file = SD.open("path.txt",FILE_WRITE);
      if(data_file)
      {
        data_file.println(data);
        data_file.flush();
        data_file.close();
        delay(3000);
      }
    }
  }
}

String solve_maze(String total_path)
{
  int len = total_path.length();
  //println(len);
  for(int i=0;i<len-2;i++)
  {
    String sub = total_path.substring(i,i+3);
    if(sub.equals("LUL"))
    {
      total_path = total_path.substring(0,i)+"S"+total_path.substring(i+3,len);
      len = total_path.length();
      i=-1;
    }
    else if(sub.equals("SUL") || sub.equals("LUS"))
    { 
      total_path = total_path.substring(0,i)+"R"+total_path.substring(i+3,len);
      len = total_path.length();
      i=-1;
    }
    else if(sub.equals("RUL"))
    { 
      total_path = total_path.substring(0,i)+"U"+total_path.substring(i+3,len);
      len = total_path.length();
      i=-1;
    }
  }
  // println(total_path);
  len = total_path.length();
  for(int i = 0;i < len;i++)
  {
    if(total_path[i] == 'R')
    {
      total_path[i] = 'L';
    }
    else if(total_path[i] == 'L')
    {
      total_path[i] = 'R';
    }
  }
  String tem = total_path;
  for(int i = 0;i < len;i++)
  {
    total_path[i] = tem[len-i-1];
  }
  return total_path;
}




















