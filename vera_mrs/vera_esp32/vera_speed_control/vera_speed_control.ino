#define ROSSERIAL_ARDUINO_TCP
#define READ_PIN(pin) ((GPIO.in >> pin) & 0x1U)

#include <Arduino.h>
#include "Motor.h"
#include "Encoder.h"
#include "IRManager.h"
#include <PID_v1.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <SH1106.h>
#include <WiFi.h>

const char* ssid = "A32D9A6";
const char* password = "dhio9090";

// Configuración OLED
#define SDA_PIN 5  
#define SCL_PIN 18

// Dirección del display, SDA, SCL
SH1106 display(0x3c, SDA_PIN, SCL_PIN);  

// Sensores IR
const int numSensors = 4;
const int sensorPins[numSensors] = {36, 39, 34, 35};
const float processVar = 1; // Varianza del proceso
const float sensorVar = 2;  // Varianza del sensor
IRManager irManager(numSensors, sensorPins, processVar, sensorVar);

// Asignacion de los pines de los motores: Configuracion
Motor motor1(5, 17, 19, 3);
Motor motor2(26, 25, 27, 1);
Motor motor3(33, 32, 23, 2);
Motor motor4(18, 16, 4, 0);

// Asignacion de los pines a los encoders 
Encoder encoder1(22, 21);
Encoder encoder2(14, 12);
Encoder encoder3(2, 13);
Encoder encoder4(0, 15);

// Variables para PWM de los motores
const int freq = 5000;
const int resolution = 8;

// Variables -> Encoder 1
volatile long encoder_value1 = 0;
uint8_t last_encoder_state1 = 0;
volatile long pulse1 = 0;

// Variables -> Encoder 2
volatile long encoder_value2 = 0;
uint8_t last_encoder_state2 = 0;
volatile long pulse2 = 0; 

// Variables -> Encoder 3
volatile long encoder_value3 = 0;
uint8_t last_encoder_state3 = 0;
volatile long pulse3 = 0; 

// Variables -> Encoder 4
volatile long encoder_value4 = 0;
uint8_t last_encoder_state4 = 0;
volatile long pulse4 = 0;

// Variables para millis 
unsigned long previous_millis = 0;
unsigned long previous_millis_ir = 0;
unsigned long previous_millis_wf = 0;
unsigned long previousCallback = 0;

// RPM
float rpm1 = 0.0;
float rpm2 = 0.0;
float rpm3 = 0.0;
float rpm4 = 0.0;

// Definición de las variables PID
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Setpoint3, Input3, Output3;
double Setpoint4, Input4, Output4;

// Variables Kp, Ki, Kd para los motores: 1, 2, 3, 4
const double kp_m1 = 0.52998; 
const double ki_m1 = 3.583;
const double kd_m1 = 0.0058866;

const double kp_m2 = 0.52998;
const double ki_m2 = 3.583;
const double kd_m2 = 0.0058866;

const double kp_m3 = 0.52998;
const double ki_m3 = 3.583;
const double kd_m3 = 0.0058866;

const double kp_m4 = 0.52998;
const double ki_m4 = 3.583;
const double kd_m4 = 0.0058866;

//Specify the links and initial tuning parameters
PID PID_M1(&Input1, &Output1, &Setpoint1, kp_m1, ki_m1, kd_m1, DIRECT);
PID PID_M2(&Input2, &Output2, &Setpoint2, kp_m2, ki_m2, kd_m2, DIRECT);
PID PID_M3(&Input3, &Output3, &Setpoint3, kp_m3, ki_m3, kd_m3, DIRECT);
PID PID_M4(&Input4, &Output4, &Setpoint4, kp_m4, ki_m4, kd_m4, DIRECT);

const float R = 0.03; // Radio de las ruedas en metros (ajustar según tu robot)
const float L = 0.07; // Distancia desde el centro al eje X en metros (ajustar)
const float W = 0.093; // Distancia desde el centro al eje Y en metros (ajustar)

// Variables de los Tiks de los encoders
double pos_act_left_front_m1 = 0;                    
double pos_act_right_front_m2 = 0;
double pos_act_right_rear_m3 = 0;                    
double pos_act_left_rear_m4 = 0;

// Constantes de optimización
const float RPM_CONVERSION_FACTOR = 60.0 / 893;
const float RPM_MULTIPLIER = 10 * RPM_CONVERSION_FACTOR;

// Server settings
IPAddress server(192, 168, 45, 14);
uint16_t serverPort = 11411;

// CallBack Subcriptor /cmd_vel
void cmd_velCallback( const geometry_msgs::Twist& msg);

ros::NodeHandle nh;

// ROS /cmd_vel
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmd_velCallback);

// ROS topic de los Tiks de los encoder de los motores: 1, 2, 3, 4
std_msgs::Int16MultiArray wheels_desired_rate_msg;
ros::Publisher wheels_desired_rate_pub("wheel_ticks", &wheels_desired_rate_msg);

// ROS topic de la velocidad en RPM de los motores: 1, 2, 3, 4
// std_msgs::Float32MultiArray motor_speeds_msg;
// ros::Publisher motor_speeds_pub("motor_speeds", &motor_speeds_msg);

std_msgs::Float32 motor1_msg;
ros::Publisher motor1_pub("motor_1" , &motor1_msg);

std_msgs::Float32 motor2_msg;
ros::Publisher motor2_pub("motor_2" , &motor2_msg);

std_msgs::Float32 motor3_msg;
ros::Publisher motor3_pub("motor_3" , &motor3_msg);
 
std_msgs::Float32 motor4_msg;
ros::Publisher motor4_pub("motor_4" , &motor4_msg);


// ROS topic de la distancia en cm de los sensores IR: 1, 2, 3, 4
std_msgs::Float32MultiArray ir_distances_msg;
ros::Publisher ir_distances_pub("ir_distances", &ir_distances_msg);


void setup()
{
  // Inicializar el display OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  // Inicializacion de la conecion Wi-Fi TCP 
  setupWiFi();
  nh.getHardware()->setConnection(server, serverPort);

  // Inicializacion del nodo. 
  nh.initNode();

  // Susbcriptores 
  nh.subscribe(sub);

  // Publicadores
  nh.advertise(wheels_desired_rate_pub);

  //nh.advertise(motor_speeds_pub);
  nh.advertise(motor1_pub);
  nh.advertise(motor2_pub);
  nh.advertise(motor3_pub);
  nh.advertise(motor4_pub);

  nh.advertise(ir_distances_pub);

  // Array para almacenar las velocidades de los motores en RPM
  wheels_desired_rate_msg.data_length = 4;
  wheels_desired_rate_msg.data = new int16_t[4];

  // Array para almacenar las velocidades de los motores en RPM
  //motor_speeds_msg.data_length = 4;
  //motor_speeds_msg.data = new float[4];

  // Array la almacenar las velocidades de los sesnores IR en cm 
  ir_distances_msg.data_length = numSensors;
  ir_distances_msg.data = new float[numSensors];

  // Inicializacion de los motores
  motor1.initMotor();
  motor2.initMotor();
  motor3.initMotor();
  motor4.initMotor();

  // Inicializacion de los encoders
  encoder1.initEncoder();
  encoder2.initEncoder();
  encoder3.initEncoder();
  encoder4.initEncoder();

  // Adjuntando interrupciones para el encoder 
  attachInterrupt(digitalPinToInterrupt(encoder1.pin_a), handle_encoder1_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1.pin_b), handle_encoder1_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2.pin_a), handle_encoder2_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2.pin_b), handle_encoder2_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3.pin_a), handle_encoder3_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3.pin_b), handle_encoder3_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4.pin_a), handle_encoder4_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4.pin_b), handle_encoder4_b, CHANGE);

  // Inicializacion de las varibales PID
  Setpoint1 = 0;
  Input1 = rpm1;
  Output1 = 0;
    
  Setpoint2 = 0;
  Input2 = rpm2;
  Output2 = 0;

  Setpoint3 = 0;
  Input3 = rpm3;
  Output3 = 0;

  Setpoint4 = 0;
  Input4 = rpm4;
  Output4 = 0;
  
  // Encendido de los controladores PID para los motores. 
  PID_M1.SetMode(AUTOMATIC);
  PID_M1.SetOutputLimits(30.0 * (255.0/500.0), 400.0 * (255.0/500.0));

  PID_M2.SetMode(AUTOMATIC);
  PID_M2.SetOutputLimits(30.0 * (255.0/500.0), 400.0 * (255.0/500.0));

  PID_M3.SetMode(AUTOMATIC);
  PID_M3.SetOutputLimits(30.0 * (255.0/500.0), 400.0 * (255.0/500.0));

  PID_M4.SetMode(AUTOMATIC);
  PID_M4.SetOutputLimits(30.0 * (255.0/500.0), 400.0 * (255.0/500.0));

  Serial.begin(115200);
}

void loop()
{
  unsigned long current_millis = millis();

  // Verificación de la conexión WiFi
  if(current_millis - previous_millis_wf >= 1000){

    previous_millis_wf = current_millis;

    if (WiFi.status() != WL_CONNECTED) {

      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
      delay(100);
      }
    }

  }

  // --------------IR Sensors-----------------

  if(current_millis - previous_millis_ir >= 200)
  {
    previous_millis_ir = current_millis;

    irManager.update();
    float* distances = irManager.getDistances();
    for (int i = 0; i < numSensors; i++) 
    {
        ir_distances_msg.data[i] = distances[i];
    }

    ir_distances_pub.publish(&ir_distances_msg);
  }

  // -------------- Motores -----------------
  
  if(current_millis - previous_millis >= 100)
  {
    previous_millis = current_millis;

    rpm1 = pulse1 * RPM_MULTIPLIER;
    rpm2 = pulse2 * RPM_MULTIPLIER;
    rpm3 = pulse3 * RPM_MULTIPLIER;
    rpm4 = pulse4 * RPM_MULTIPLIER;
    
    Input1 = rpm1;
    PID_M1.Compute();
    ledcWrite(motor1.pwm_channel, 200);
    //motor_speeds_msg.data[0] = rpm1;
    motor1_msg.data = rpm1;

    Input2 = rpm2;
    PID_M2.Compute();
    ledcWrite(motor2.pwm_channel, 200);
    //motor_speeds_msg.data[1] = rpm2;
    motor2_msg.data = rpm2;

    Input3 = rpm3;
    PID_M3.Compute();
    ledcWrite(motor3.pwm_channel, 200);
    //motor_speeds_msg.data[2] = rpm3;
    motor3_msg.data = rpm3;   

    Input4 = rpm4;
    PID_M4.Compute();
    ledcWrite(motor4.pwm_channel, 200);
    //motor_speeds_msg.data[3] = rpm4;
    motor4_msg.data = rpm4;   

    updatePositions();
    resetPulses();

    publishPos();
    motor1_pub.publish(&motor1_msg);
    motor2_pub.publish(&motor2_msg);
    motor3_pub.publish(&motor3_msg);
    motor4_pub.publish(&motor4_msg);

    nh.spinOnce();
  }
}

void setupWiFi()
{
  display.clear();
  display.drawString(15, 20, "Connecting to ROS");
  display.display();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(5000);
  }
  
  display.clear();
  display.drawString(0, 10, "Connected to:");
  display.drawString(0, 20, WiFi.SSID());
  display.drawString(0, 30, "IP: " + WiFi.localIP().toString());
  display.display();
  delay(2000);

  display.clear();

  // Configuración de los ojos
  int eyeWidth = 40;
  int eyeHeight = 20;
  int eyeDistance = 20;
  int eyeY = (display.getHeight() / 2) - (eyeHeight / 2); 
  int centerX = display.getWidth() / 2;

  for (int i = 0; i < eyeWidth; i++) {
    int height = eyeHeight * (1 - pow(2.0 * i / eyeWidth - 1, 2));
    int topY = eyeY + (eyeHeight - height) / 2;
    display.drawLine(centerX - eyeDistance / 2 - i, topY, centerX - eyeDistance / 2 - i, topY + height);
    display.drawLine(centerX + eyeDistance / 2 + i, topY, centerX + eyeDistance / 2 + i, topY + height);
  }

  display.display();
}

void cmd_velCallback( const geometry_msgs::Twist& msg)
{
  unsigned long current_millis_cb = millis();
  
  if(current_millis_cb - previousCallback >= 100){
    previousCallback = current_millis_cb;

    if (msg.linear.x == 0.0 && msg.linear.y == 0.0 && msg.angular.z == 0.0) 
    {
      // Detener todos los motores
      Setpoint1 = 0;
      Setpoint2 = 0;
      Setpoint3 = 0;
      Setpoint4 = 0;

      // Asegurarse de que los PWM estén en 0
      ledcWrite(motor1.pwm_channel, 0);
      ledcWrite(motor2.pwm_channel, 0);
      ledcWrite(motor3.pwm_channel, 0);
      ledcWrite(motor4.pwm_channel, 0);

      digitalWrite(motor1.pin_a, LOW);
      digitalWrite(motor1.pin_b, LOW);

      digitalWrite(motor2.pin_a, LOW);
      digitalWrite(motor2.pin_b, LOW);

      digitalWrite(motor3.pin_a, LOW);
      digitalWrite(motor3.pin_b, LOW);

      digitalWrite(motor4.pin_a, LOW);
      digitalWrite(motor4.pin_b, LOW);
    
      return;  // Salir del callback
    }

    //
    float vx = msg.linear.x;
    float vy = msg.linear.y;
    float wz = msg.angular.z;

    // Modelo cinematico para un robot omnidireccional con ruedas mecanum
    float v1 = vx - vy - wz*(L + W);
    float v2 = vx + vy + wz*(L + W);
    float v3 = vx - vy + wz*(L + W);
    float v4 = vx + vy - wz*(L + W);

    // Convertir las velocidades a RPM
    rpm1 = linearToRPM(v1, R);
    rpm2 = linearToRPM(v2, R);
    rpm3 = linearToRPM(v3, R);
    rpm4 = linearToRPM(v4, R);

    // Establecer la dirección de cada motor
    setMotorDirection(motor1, v1);
    setMotorDirection(motor2, v2);
    setMotorDirection(motor3, v3);
    setMotorDirection(motor4, v4);

    // Valor absoluto de las RPM como setpoint para el PID
    Setpoint1 = abs(rpm1);
    Setpoint2 = abs(rpm2);
    Setpoint3 = abs(rpm3);
    Setpoint4 = abs(rpm4);
  }
}

// Funcion para convertir velocidad lineal a RPM
float linearToRPM(float v, float R)
{
  return v * 60 / (2 * PI * R);
}

//  Funcion para determinar la direccion de los motores en funciona al modelo cinematico. 
void setMotorDirection(Motor &motor, float speed) {
  if (speed > 0) {
    digitalWrite(motor.pin_a, HIGH);
    digitalWrite(motor.pin_b, LOW);
  } else {
    digitalWrite(motor.pin_a, LOW);
    digitalWrite(motor.pin_b, HIGH);
  }
}

// Funcion para publicar los Ticks de los encoders 
void publishPos() {

  wheels_desired_rate_msg.data[0] = pos_act_left_front_m1;
  wheels_desired_rate_msg.data[1] = pos_act_right_front_m2;
  wheels_desired_rate_msg.data[2] = pos_act_right_rear_m3;
  wheels_desired_rate_msg.data[3] = pos_act_left_rear_m4;

  wheels_desired_rate_pub.publish(&wheels_desired_rate_msg);
}

//Reset Pulsos
void resetPulses() {
    pulse1 = 0;
    pulse2 = 0;
    pulse3 = 0;
    pulse4 = 0;
}

// Actualizar posicion
void updatePositions() {
    pos_act_left_front_m1 = encoder_value1;
    pos_act_right_front_m2 = encoder_value2;
    pos_act_right_rear_m3 = encoder_value3;
    pos_act_left_rear_m4 = encoder_value4;
}

// Tick con preccion cuadruple de los encoders (Motor 1)

void update_encoder(volatile long &encoder_value, uint8_t &last_encoder_state, int pin_a, int pin_b, volatile long &pulse) {
  uint8_t state = 0;

  if (READ_PIN(pin_a)) bitSet(state, 0);
  if (READ_PIN(pin_b)) bitSet(state, 1);

  if (last_encoder_state == 2 && state == 0) encoder_value++;
  if (last_encoder_state == 0 && state == 1) encoder_value++;
  if (last_encoder_state == 3 && state == 2) encoder_value++;
  if (last_encoder_state == 1 && state == 3) encoder_value++;

  if (last_encoder_state == 1 && state == 0) encoder_value--;
  if (last_encoder_state == 3 && state == 1) encoder_value--;
  if (last_encoder_state == 0 && state == 2) encoder_value--;
  if (last_encoder_state == 2 && state == 3) encoder_value--;

  pulse++;
  last_encoder_state = state;
}
//----------------------------------------------------------------------------------------------------------------------------
void handle_encoder_a(volatile long &encoder_value, uint8_t &last_encoder_state, int pin_a, int pin_b, volatile long &pulse) {
    update_encoder(encoder_value, last_encoder_state, pin_a, pin_b, pulse);
}

void handle_encoder_b(volatile long &encoder_value, uint8_t &last_encoder_state, int pin_a, int pin_b, volatile long &pulse) {
    update_encoder(encoder_value, last_encoder_state, pin_a, pin_b, pulse);
}

//------------------------------------------------------------------------------------------------
void handle_encoder1_a() {
    handle_encoder_a(encoder_value1, last_encoder_state1, encoder1.pin_a, encoder1.pin_b, pulse1);
}

void handle_encoder1_b() {
    handle_encoder_b(encoder_value1, last_encoder_state1, encoder1.pin_a, encoder1.pin_b, pulse1);
}
//------------------------------------------------------------------------------------------------
void handle_encoder2_a() {
    handle_encoder_a(encoder_value2, last_encoder_state2, encoder2.pin_a, encoder2.pin_b, pulse2);
}

void handle_encoder2_b() {
    handle_encoder_b(encoder_value2, last_encoder_state2, encoder2.pin_a, encoder2.pin_b, pulse2);
}
//------------------------------------------------------------------------------------------------
void handle_encoder3_a() {
    handle_encoder_a(encoder_value3, last_encoder_state3, encoder3.pin_a, encoder3.pin_b, pulse3);
}

void handle_encoder3_b() {
    handle_encoder_b(encoder_value3, last_encoder_state3, encoder3.pin_a, encoder3.pin_b, pulse3);
}
//------------------------------------------------------------------------------------------------
void handle_encoder4_a() {
    handle_encoder_a(encoder_value4, last_encoder_state4, encoder4.pin_a, encoder4.pin_b, pulse4);
}

void handle_encoder4_b() {
    handle_encoder_b(encoder_value4, last_encoder_state4, encoder4.pin_a, encoder4.pin_b, pulse4);
}
