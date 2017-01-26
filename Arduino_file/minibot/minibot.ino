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
int forward_speed = 90;
int forward_speed_for_straight = 70;
int left_speed = 80;
int right_speed = 80;
int u_speed = 60;
int backward_speed = 100;
int l_delay = 200;
float speed_ratio_n = .8;
float speed_ratio_l = .5;
unsigned char Light_sensor_data = 0;//to store the sensor value
unsigned char Light_sensor_data_back = 0;
// to indicate left turn
int thres_val_mid = 70;
int mid_pin = 10;

double distance;//to store the distance of any object
double threshold_distance = 5;//distance from the destination box
//buzzer pin
int buzzer_pin = 21;
//all sensor input pin
int total_sensor = 10;//no of sensors
int white[10] = {
  70,70,70,70,70,70,70,70,70,70};//referance for white or black
int left1 = 0;
int left2 = 1;
int middle = 2;
int right2 = 3;
int right1 = 4;
int back_left= 5;
int back_middle = 6;
int back_right = 7;

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
  init_sd();
  delay(3000);

}

void loop()
{
  //flow_line();
  //goto_node();
  //analog_go_forward(forward_speed);
   //check_sensor_data();
  //Serial.println("done");
  //do_the_job();
  navigate_arena();
  //analog_go_left_n(60);
  //path = "i am mrinmoy";
   write_sd(path);
  //read_sd();
 while(1)
 check_sensor_data();
  //delay(30000);
}


void do_the_job()
{
  navigate_arena();
  Serial.println(path);
  short_path = solve_maze(path);
  Serial.println(short_path);
  delay(10000);
  go_through_shortest_path();
}

void goto_node()
{
  unsigned char sensor_data = 0;
  unsigned char stop_command =0;
  unsigned char forward_sensor = 0;
  while(stop_flag)
  {
    get_sensor_data();
    forward_sensor = Light_sensor_data & 0b00011111;
    if(Light_sensor_data == 0b11111111 && Light_sensor_data_back == 0b00000011)
    {
      stop_motor();
      break;
    }
    sensor_data = ((Light_sensor_data & 0b11100000)>>4) | (Light_sensor_data_back & 0b00000001) | ((Light_sensor_data_back & 0b00000010)<<3);
    stop_command = sensor_data & 0b00011111;
//    if(forward_sensor != 0b00011111 || forward_sensor != 0b00000000)
//    {
//      flow_line();
//    }
    if(sensor_data == 0b00000100 || sensor_data == 0)//|| sensor_data == 0b00001100 || sensor_data == 0b00000110)
    {
      analog_go_forward(forward_speed_for_straight);
    }
    else if(sensor_data == 0b00001000 || sensor_data == 0b00001100)
    {
      analog_go_right_n(forward_speed_for_straight,speed_ratio_n);
    }
    else if(sensor_data == 0b00000010 || sensor_data == 0b00000110)
    {
      analog_go_left_n(forward_speed_for_straight,speed_ratio_n);
    }
    if((stop_command & 0b00010000)||(stop_command & 0b00000001)||stop_command == 0b00011000 || stop_command == 0b00000011 || (Light_sensor_data == 0b00000000)||((stop_command & 0b00011100) == 0b00011100) || stop_command == 0b00011111 || ((stop_command & 0b00000111) == 0b00000111))
    {
      stop_motor();
      break;
    }
    end_point_check();
  }
  get_sensor_data();
  if(Light_sensor_data & 0b00011111 == 0b00011111)
  {
    stop_motor();
    stop_flag =0;
  }
}

void navigate_arena()
{
  while(stop_flag)
  {
    //    get_sensor_data();
    //    if( fl == 1 ||(Light_sensor_data == 0b11111111 && (Light_sensor_data_back == 0b00000011 || Light_sensor_data_back == 0b00000010 || Light_sensor_data_back == 0b00000001)))
    //    {
    //      stop_motor();
    //      break;
    //    }
    flow_line();
    unsigned char straight_sensor_data = 0;
    unsigned char left_sensor_data = 0;
    unsigned char sensor_data = 0;
    get_sensor_data();
    sensor_data = Light_sensor_data & 0b00011111;
    straight_sensor_data = Light_sensor_data & 0b00011100;
    left_sensor_data = Light_sensor_data & 0b00000111;
    if((left_sensor_data == 0b00000111) ||(left_sensor_data == 0b00000011) ||(left_sensor_data == 0b00000001)) // || (left_sensor_data == 0b00000101)|| (left_sensor_data == 0b00000010)
    {
      //  stop_motor();
      Serial.println("in left");
      stop_motor();
      //check_sensor_data();
      left_turn();
      //Serial.println("after left");
      //check_sensor_data();
      delay(100);

    }
    else if((straight_sensor_data == 0b00011100) ||(straight_sensor_data == 0b00011000)||(straight_sensor_data == 0b00010000))//||(straight_sensor_data == 0b00001000)|| (straight_sensor_data == 0b00010100)
    {
      //stop_motor();
      Serial.println("in straight");
      stop_motor();
      //check_sensor_data();
      straight();
      delay(100);
    }
    else if(sensor_data == 0b00000000 && check_mid() == 0)
    {
      Serial.println("in U");
      stop_motor();
      u_turn();
      delay(50);
    }
    end_point_check();
  }
  Serial.println("OUT Of ARENA");
}

void left_turn()
{
  unsigned char sensor_data = 0;
  goto_node();
  fl = 0;
  get_sensor_data();
  if(Light_sensor_data == 0b11111111 && (Light_sensor_data_back == 0b00000011 || Light_sensor_data_back == 0b00000010 || Light_sensor_data_back == 0b00000001))
  {
    fl=1;
  }
  if(fl == 0)
  {
    get_sensor_data();
    sensor_data = Light_sensor_data & 0b00011111;
    //if(sensor_data != 0b00000000)
    while(stop_flag)
    {
      get_sensor_data();
      sensor_data = Light_sensor_data & 0b00001110;
      if(sensor_data == 0b00000000 )//&& check_mid() == 0)//&& Light_sensor_data_back == 0)
      {
        stop_motor();
        //Serial.println("in ck mid");
        //delay(3000);
        break;
      }
      else
      {
        analog_go_u_left(u_speed);
      }
      end_point_check();
    }


    //  int flagsr = 0;
    if(stop_flag == 1)
    {
      analog_go_u_left(u_speed);
      delay(l_delay);
      stop_motor();
    }
    while(stop_flag)
    {
      get_sensor_data();
      sensor_data = Light_sensor_data & 0b00011111;
      if(sensor_data == 0b00000100 || sensor_data == 0b00000110||sensor_data == 0b00001100)
      {
        stop_motor();
        path += "L";
        break;
      }
      else
      {
        //go_left();
        analog_go_u_left(u_speed);
      }
      end_point_check();
    }
  }
}

void come_to_straight()
{
  unsigned char sensor_data = 0;
  unsigned char stop_command =0;
  while(stop_flag)
  {
    get_sensor_data();
    sensor_data = ((Light_sensor_data & 0b11100000)>>4) | (Light_sensor_data_back & 0b00000001) | ((Light_sensor_data_back & 0b00000010)<<3);
    stop_command = sensor_data & 0b00000111;
    if(stop_command == 0b00000111)
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_right(u_speed);
    }
    end_point_check();
  }
}

void straight()
{
  unsigned char sensor_data = 0;
  int flagg = 0;
  while(stop_flag)
  {
    boolean left_check = false;
    get_sensor_data();
    if( Light_sensor_data == 0b11111111)
    {
      stop_motor();
      break;
    }
    left_check = (Light_sensor_data & 0b00000001) || (Light_sensor_data_back & 0b0000001) ;
    if(left_check)
    {
      flagg = 1;
      stop_motor();
      left_turn();
      break;
    }
    sensor_data = ((Light_sensor_data & 0b11100000)>>4) | (Light_sensor_data_back & 0b00000001) | ((Light_sensor_data_back & 0b00000010)<<3);
    unsigned char stop_command = sensor_data & 0b00011111;
    if(sensor_data == 0b00000100)
    {
      analog_go_forward(forward_speed_for_straight);
    }
    else if(sensor_data == 0b00001100 || sensor_data == 0b00001000)
    {
      analog_go_right_n(forward_speed_for_straight,speed_ratio_n);
    }
    else if(sensor_data == 0b00000110 || sensor_data == 0b00000010)
    {
      analog_go_left_n(forward_speed_for_straight,speed_ratio_n);
    }
    else if((stop_command & 0b00010000)||(stop_command & 0b00000001)||(stop_command == 0b00000000)||((stop_command & 0b00011100) == 0b00011100) || stop_command == 0b00011111 || ((stop_command & 0b00000111) == 0b00000111))
    {
      stop_motor();
      break;
    }
    end_point_check();
  }
  get_sensor_data();
  if((Light_sensor_data == 0b11111111 && (Light_sensor_data_back == 0b00000011 || Light_sensor_data_back == 0b00000010 || Light_sensor_data_back == 0b00000001)))
  {
    stop_motor();
    flagg = 1;
  }
  if(Light_sensor_data & 0b00011111 == 0b00011111)
  {
    stop_motor();
    stop_flag =0;
  }
  if(flagg == 0)
  {
    while(stop_flag)
    {
      get_sensor_data();
      sensor_data = Light_sensor_data & 0b00011111;
      if((sensor_data == 0b00001100) || (sensor_data == 0b00000100) || (sensor_data == 0b00000110) || sensor_data > 0)
      {
        stop_motor();
        path += "S";
        break;
      }
      else if( sensor_data == 0b00000000 && check_mid() == 0)
      {
        stop_motor();
        right_turn();
        break;
      }
      else
      {
        //go_forward();
        //analog_go_forward(forward_speed);
      }
      end_point_check();
    } 
  }
}

void u_turn()
{
  unsigned char sensor_data = 0;
  goto_node();
  while(stop_flag)
  {
    get_sensor_data();
    sensor_data = Light_sensor_data & 0b00011111;
    if((sensor_data == 0b00000100) ||(sensor_data == 0b00001100)|| (sensor_data == 0b00000110) || (sensor_data & 0b00001000)) //&& (sensor_data == 0b00000100))) //||(sensor_data == 0b00001100))
    {
      stop_motor();
      path += "U";
      break;
    }
    else
    {
      //go_u_left();
      analog_go_u_left(u_speed);
    }
    end_point_check();
  }
}

void right_turn()
{
  unsigned char sensor_data = 0;
  while(stop_flag)
  {
    get_sensor_data();
    if( Light_sensor_data == 0b11111111)
    {
      stop_motor();
      break;
    }
    sensor_data = Light_sensor_data & 0b00011111;
    if(sensor_data == 0b00000100 ||sensor_data == 0b00000110 )//|| sensor_data == 0b00001100)
    {
      path += "R";
      stop_motor();
      break;
    }
    else
    {
      //go_right();
      analog_go_u_right(u_speed);
    }
    end_point_check();
  }  
}

void go_through_shortest_path()
{
  int indx = 0;
  int len = short_path.length();
  u_turnn();
  while(1)
  {
    stop_flag = 1;
    flow_line();
    unsigned char sensor_data = 0;
    get_sensor_data();
    sensor_data = Light_sensor_data & 0b00011111;
    if((sensor_data == 0b00011111)||(sensor_data == 0b00001111)|| (sensor_data == 0b00000111)||(sensor_data == 0b00000011)||(sensor_data == 0b0000001)||(sensor_data == 0b00011110)||(sensor_data == 0b00011100)||(sensor_data == 0b00011000)||(sensor_data == 0b00010000))
    {
      if(short_path[indx] == 'R')
      {
        goto_node();
        right_turnn();
        indx++;
      }
      else if(short_path[indx] == 'L')
      {
        left_turnn();
        indx++;
      }
      else if(short_path[indx] == 'S')
      {
        goto_node();
        //straight();
        indx++;
      }
    }
    if(indx == len)
    {
      stop_motor();
      break;
    }
  }
  while(1)
  {
    flow_line();
    get_sensor_data();
    if(Light_sensor_data == 0b00000000 || Light_sensor_data == 0b11111111 || (Light_sensor_data & 0b00011111) == 0)
    {
      stop_motor();
      break;
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



void flow_line()
{
  unsigned char sensor_data = 0;
  get_sensor_data();
  sensor_data = Light_sensor_data & 0b00001110;//0b00011111
  boolean b = check_mid() && (Light_sensor_data & 0b10000000 || Light_sensor_data & 0b01000000 || Light_sensor_data & 0b00100000);
  if(sensor_data == 0b00000100 || sensor_data == 0b00001110 )//|| b)
  {
    //go_forward();
    analog_go_forward(forward_speed);
  }
  else if((sensor_data == 0b00001000) || (sensor_data == 0b00001100))
  {
    //go_right();
    analog_go_right_n(right_speed,speed_ratio_l);
  }
  else if((sensor_data == 0b00000010) || (sensor_data == 0b00000110))
  {
    //go_left();
    analog_go_left_n(left_speed,speed_ratio_l);
  }
  else if((sensor_data == 0b00011111)||(sensor_data == 0b00011110)||(sensor_data == 0b00011100)||(sensor_data == 0b00011000) || (sensor_data == 0b00001111) ||(sensor_data == 0b00000111) ||(sensor_data == 0b00000011))
  {
    stop_motor();
  }
  if( Light_sensor_data == 0b11111111 || Light_sensor_data == 0)
  {
    stop_motor();
  }
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
void go_right()
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,HIGH);
  digitalWrite(right_motor_forward,LOW);
}
void analog_go_right(int speed_m)
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  analogWrite(left_motor_forward,speed_m);
  digitalWrite(right_motor_forward,LOW);
}
void analog_go_right_n(int speed_m,float speed_ratio)
{

  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  analogWrite(left_motor_forward,speed_m);
  analogWrite(right_motor_forward,int(speed_m * speed_ratio));
}
void go_backright()
{
  digitalWrite(left_motor_backward,HIGH);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}
void analog_go_backright(int speed_m)
{
  analogWrite(left_motor_backward,speed_m);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}

void go_u_right()
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,HIGH);
  digitalWrite(left_motor_forward,HIGH);
  digitalWrite(right_motor_forward,LOW); 
}
void analog_go_u_right(int speed_m)
{
  digitalWrite(left_motor_backward,LOW);
  analogWrite(right_motor_backward,speed_m);
  analogWrite(left_motor_forward,speed_m);
  digitalWrite(right_motor_forward,LOW); 
}
void go_left()
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,HIGH);
}
void analog_go_left(int speed_m)
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  analogWrite(right_motor_forward,speed_m);
}
void analog_go_left_n(int speed_m,float speed_ratio)
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  analogWrite(left_motor_forward,int(speed_ratio*speed_m));
  analogWrite(right_motor_forward,speed_m);
}
void go_backleft()
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,HIGH);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}
void analog_go_backleft(int speed_m)
{
  digitalWrite(left_motor_backward,LOW);
  analogWrite(right_motor_backward,speed_m);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}
void go_u_left()
{
  digitalWrite(left_motor_backward,HIGH);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,HIGH);
}
void analog_go_u_left(int speed_m)
{
  analogWrite(left_motor_backward,speed_m);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,LOW);
  analogWrite(right_motor_forward,speed_m);
}
void go_forward()
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  digitalWrite(left_motor_forward,HIGH);
  digitalWrite(right_motor_forward,HIGH);
}
void analog_go_forward(int speed_m)
{
  digitalWrite(left_motor_backward,LOW);
  digitalWrite(right_motor_backward,LOW);
  analogWrite(left_motor_forward,speed_m);
  analogWrite(right_motor_forward,speed_m);
}

void go_backward()
{
  digitalWrite(left_motor_backward,HIGH);
  digitalWrite(right_motor_backward,HIGH);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
}

void analog_go_backward(int speed_m)
{
  analogWrite(left_motor_backward,speed_m);
  analogWrite(right_motor_backward,speed_m);
  digitalWrite(left_motor_forward,LOW);
  digitalWrite(right_motor_forward,LOW);
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

void get_sensor_data()
{
  Light_sensor_data = 0;
  Light_sensor_data_back = 0;
  int analog_data = 0;
  for(int i = 0; i < total_sensor; i++)
  {
    analog_data = analogRead(i);
    //Serial.println(analog_data);
    if(i<8)
    {
      if(analog_data > white[i])
      {
        Light_sensor_data |= 1<<(i);
      }
    }
    else
    {
      if(analog_data > white[i])
      {
        Light_sensor_data_back |= 1<<(i-8);
      }  
    }
    delay(1);
  }
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
void get_sensor_data_ck()
{
  Light_sensor_data = 0;
  Light_sensor_data_back = 0;
  int analog_data = 0;
  for(int i = 0; i < total_sensor; i++)
  {
    analog_data = analogRead(i);
    Serial.println(analog_data);
    if(i<8)
    {
      if(analog_data > white[i])
      {
        Light_sensor_data |= 1<<(i);
      }
    }
    else
    {
      if(analog_data > white[i])
      {
        Light_sensor_data_back |= 1<<(i-8);
      }  
    }
    delay(1);
  }
}

void check_sensor_data()
{
  get_sensor_data_ck();
  Serial.println("****************************");
  Serial.println(Light_sensor_data,BIN);
  Serial.println(Light_sensor_data_back,BIN);
  Serial.println("mid:");
  Serial.println(check_mid_ck());
  Serial.println("****************************");
  delay(3000);
}

void calibrate_sensor()
{
  int thresh_value[total_sensor];
  blow_buzzer(30);
  delay(10000);
  for(int i = 0; i < total_sensor ; i++)
  {
    thresh_value[i] = analogRead(i);
    delay(5);
  }
  blow_buzzer(30);
  delay(10000);
  for(int i = 0; i < total_sensor ; i++)
  {
    thresh_value[i] += analogRead(i);
    delay(5);
  }
  for(int i = 0; i < total_sensor ; i++)
  {
    white[i] = thresh_value[i]/2;
  }
  blow_buzzer(1000);
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

void end_point_check()
{
  get_sensor_data();
  boolean t = (Light_sensor_data == 0b11111111 && check_mid()==1) || (Light_sensor_data == 0b11111111 && (Light_sensor_data_back == 0b00000011 || Light_sensor_data_back == 0b00000001 || Light_sensor_data_back == 0b00000010));
  if(t)
  {
    stop_flag = 0;
    stop_motor();
  }
}


void left_turnn()
{
  unsigned char sensor_data = 0;
  goto_node();
  //if(sensor_data != 0b00000000)
  while(1)
  {
    get_sensor_data();
    sensor_data = Light_sensor_data & 0b00001110;
    if(sensor_data == 0b00000000 )//&& check_mid() == 0)//&& Light_sensor_data_back == 0)
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_left(u_speed);
    }
  }
  analog_go_u_left(u_speed);
  delay(l_delay);
  stop_motor();
  while(1)
  {
    get_sensor_data();
    sensor_data = Light_sensor_data & 0b00011111;
    if(sensor_data == 0b00000100 || sensor_data == 0b00000110||sensor_data == 0b00001100)
    {
      stop_motor();
      break;
    }
    else
    {
      //go_left();
      analog_go_u_left(u_speed);
    }
  }
}


void right_turnn()
{
  unsigned char sensor_data = 0;
  while(1)
  {
    get_sensor_data();
    sensor_data = Light_sensor_data & 0b00000111;
    if(sensor_data == 0b00000000)
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_right(u_speed);
    }
  }
  analog_go_u_right(u_speed);
  delay(l_delay);
  stop_motor();
  while(1)
  {
    get_sensor_data();
    sensor_data = Light_sensor_data & 0b00011111;
    if(sensor_data == 0b00000100 ||sensor_data == 0b00000110 )//|| sensor_data == 0b00001100)
    {
      stop_motor();
      break;
    }
    else
    {
      //go_right();
      analog_go_u_right(u_speed);
    }
  }  
}


void u_turnn()
{
  unsigned char sensor_data = 0;
  while(1)
  {
    get_sensor_data();
    sensor_data = Light_sensor_data & 0b00001110;
    if(sensor_data == 0b00000000 )//&& check_mid() == 0)//&& Light_sensor_data_back == 0)
    {
      stop_motor();
      break;
    }
    else
    {
      analog_go_u_left(u_speed);
    }
  }
  analog_go_u_left(u_speed);
  delay(l_delay);
  stop_motor();
  while(1)
  {
    get_sensor_data();
    sensor_data = Light_sensor_data & 0b00011111;
    if((sensor_data == 0b00000100) ||(sensor_data == 0b00001100)|| (sensor_data == 0b00000110) || (sensor_data & 0b00001000)) //&& (sensor_data == 0b00000100))) //||(sensor_data == 0b00001100))
    {
      stop_motor();
      break;
    }
    else
    {
      //go_u_left();
      analog_go_u_left(u_speed);
    }
  }
}




















