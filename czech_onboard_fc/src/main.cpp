#include "Arduino.h"
#include "Arduino_BMI270_BMM150.h"
#include "Arduino_LPS22HB.h"
#include "Arduino_LSM9DS1.h"
#include "SPI.h"
#include "LoRa.h"
#include "SD.h"
#include "Adafruit_BMP280.h"
#include "Servo.h"

//float vertical_velocity = (current_altitude - last_altitude) / LOOP_FREQUENCY * 1000.0;



/////////////////////////////////// LAUNCH DETECTION PARAMETERS

  const float LAUNCH_ACCEL_THRESHOLD = 2.5; // g
  const unsigned long LAUNCH_MIN_DURATION = 100; // ms



/////////////////////////////////// Height parameters

  #define SEAL_LEVEL_PRESSURE 1013.25

  float last_altitude = 0;
  float current_altitude = 0;



/////////////////////////////////// Parachute trigger

#define SERVO_PIN 3  // Connect to MOSFET controlling e-match or pwm for servo

bool deployed_parachute = false; // Default state
Servo deployServo;

// main deployment
#define SERVO_ROTATION 90 // in degrees of 360 rotation
#define SERVO_MOVEMENT_RESERT_DELAY 3000 // in miliseconds

// seconary deployent
#define MAX_TIME_TO_DEPLOY 40 // seconds


/////////////////////////////////// SERIAL COMMUNICATION

  #define SERIAL_BAUD_RATE 9600



/////////////////////////////////// GRACEFUL RECOVERY INITIALIZATION

  // Graceful recovery for the initialization of the modules
  #define MAX_INITIALIZATION_RETRIES 3   // Maximum number of retry attempts for each sensor
  #define INITIALIZATION_RETRY_DELAY 2000  // Delay between retries in milliseconds



/////////////////////////////////// Fight state

  enum FlightState {IDLE, ASCENT, APOGEE, DESCENT, LANDED};
  FlightState rocket_state = IDLE; // Deafult state
  bool launched = false;
  


/////////////////////////////////// Flight timing

  // time since ascent started
  unsigned long launchTime;  // miliseconds



/////////////////////////////////// FLIGHT DATA

  // Accelerometer continuely update variables
  float acc_x = 0.0;
  float acc_y = 0.0;
  float acc_z = 0.0;

  // Gyroscope continuely update variables
  float gyro_x = 0.0;
  float gyro_y = 0.0;
  float gyro_z = 0.0;

  // Barometer continuesly update variables
  float pressure = 0.0;
  float temperature = 0.0;



/////////////////////////////////// BUZZER

  #define BUZZER_PIN 5  // on the fly debuging / on site waiting beep



/////////////////////////////////// SD

  #define SD_MODULE_CS_PIN 4  // Sd module spi select pin

  // Sd files
  #define SD_ALL_DATA_FILE_NAME "flightdata.csv" 
  File flightdata_file;



/////////////////////////////////// LORA

  #define LORA_FREQENCY 433E6  // loRa frequency for transmiting (433hz)
  int LORA_SYNC_WORD = 0xF3;  // set channel for LoRa module communication
  int LORA_SET_GAIN = 0;  // for setting transmition power (in dB)

  // Identifiers for understanding what we are sending
  byte LORA_HEADER = 0xA4;

  #define LOOP_FREQUENCY 20 // 50hz



/////////////////////////////////// BMP280

  Adafruit_BMP280 BMP280; // I2C Interface















/////////////////////////////////// Functions

///// Other

void buzzer_chime(int repeat, int time_on, int time_off); // buzzer function
void rgb_led_color(int color); // set rgb led color
void rgb_led_blink(int color, int repeat, int time_on, int time_off); // (0 red, 1 green, 2 blue, 3 white, 4 OFF), repeat time "blink times", (time on), (time off)

void checkForCommands();



///// Pre-flight

  // Graceful recovery in case of failed sensor initialization
  int initialize_sensor_with_retry(int (*sensor_init_func)(), const char* sensor_name);

  // Initialization functions
  int initialize_BMI270_BMM150();
  int initialize_barometer();
  int initialize_lora();
  int initialize_sd_card_module();
  int initialize_bmp280();

  // Waiting mode
  void enter_armed_mode();



///// During flight

  // Update sensor variables
  void update_accelerometer_data();
  void update_gyro_data();
  void update_barometer_data();
  void update_height_data();
 

  // check armed mode
  void checkLaunchCondition(unsigned long current_time);

  //if lanche state is true save data to sd
  void save_data_to_sd(int post_launch_time);

  // if launch state is true start sending telemetry data over lora
  void sendTelemetry(int time);



///// ASCENT


float calculateAltitudeFromPressure(float pressure, float seaLevelPressure);
void check_height();



///// APOGEE

void deploy_parachute(int rotate, int time);




///// DECENT

void battery_preserve();




///// LANDED

void ping_ground_station();







void setup() {

  //initialize serial connection
  Serial.begin(SERIAL_BAUD_RATE);

  //initialize pins
  pinMode(BUZZER_PIN, OUTPUT);  // buzzer for debuging

  //Arduino's built-in rgb led
  pinMode(LEDR, OUTPUT); 
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);


  // Parachute deployment initialization
  pinMode(SERVO_PIN, OUTPUT);
  deployServo.attach(SERVO_PIN);
  deployServo.write(0);  // Initial locked position
  digitalWrite(SERVO_PIN, LOW);  // Safe state

  //Initialize sensors

  initialize_sensor_with_retry(initialize_BMI270_BMM150, "Built-in IMU");
  initialize_sensor_with_retry(initialize_barometer, "Bult-in BAROMETER");
  initialize_sensor_with_retry(initialize_lora, "Lora module");
  initialize_sensor_with_retry(initialize_sd_card_module, "SD card ");
  initialize_sensor_with_retry(initialize_bmp280, "BMP-280");

  //for (int i = 0; i != (sizeof(initialization_status_of_modules) / sizeof(initialization_status_of_modules[0])); i++){
  //if (initialization_status_of_modules[i] == 1){
  //  switch(i){
  //    case 0:
  //      Serial.println("Failed to initialize IMU");
  //      break;
  //    case 1:
  //      Serial.println("Failed to initialize barometer");
  //      break;
  //      case 2:
  //      Serial.println("Failed to initialize LoRa module");
  //      break;
  //      case 3:
  //      Serial.println("Failed to initialize SD card module");
  //      break;
  //      case 4:
  //      Serial.println("Failed to initialize BMP-280");
  //      break;
  //    default:
  //      Serial.println("Unknown Error initializing modules");
  //      break;
  //   }
  //   buzzer_chime(3, 500, 50); // if initialization fails
  //   //while (true);  // Halt the program if initialization fails
  //  }
  //}

  delay(5000); // Time after initialization to enter armed mode

  enter_armed_mode(); //enter flight wait mode
}



void loop() {
  
  


  //static unsigned long lastLoop = millis();
  unsigned long now = millis();

  update_accelerometer_data();
  update_barometer_data();
  update_gyro_data();
  update_height_data();
  sendTelemetry(now);


  switch (rocket_state) {
    case IDLE:
        checkLaunchCondition(now);
      break;

    case ASCENT:
        check_height();
        
      break;

    case APOGEE:
      if (!deployed_parachute) {
        deploy_parachute(SERVO_ROTATION, SERVO_MOVEMENT_RESERT_DELAY);
        
      }
      save_data_to_sd(now);
      break;

    case DESCENT:
      //if (altitude < baseAltitude + 5) {
      //  rocket_state = LANDED;
      //  Serial.println("Landing detected");
      //}
      save_data_to_sd(now);
      break;

    case LANDED:
      // Flight complete
      break;
  }

    delay(LOOP_FREQUENCY);
  

}

/////////////////////////////////////////////// Other

void buzzer_chime(int repeat, int time_on, int time_off){
  for (int i = 0; i < repeat; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(time_on);
    digitalWrite(BUZZER_PIN, LOW);
    delay(time_off);
  }
}

void rgb_led_color(int color){
  
  if (color == 0) { // RED
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
  }
  else if (color == 1){ //GREEN
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, HIGH);
  } 
  else if (color == 2){ //BLUE
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, LOW);
  }
  else if (color == 3){ //WHITE
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDB, LOW);
  }
  else if (color == 4){ // NO COLOR / OFF
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
  }

}

void rgb_led_blink(int color, int repeat, int time_on, int time_off){
  for (int i = 0; i < repeat; i++){
    rgb_led_color(color);
    delay(time_on);
    rgb_led_color(4);
    delay(time_off);
  }
}


//void checkForCommands(){
//
//int packetSize = LoRa.parsePacket();
//  if (packetSize > 0) {
//    String cmd = "";
//    while (LoRa.available()) {
//      cmd += (char)LoRa.read();
//    }
//    cmd.trim();
//    Serial.print("Command received: "); Serial.println(cmd);
//
//    if (cmd == "CMD_DEPLOY" && !deployed) {
//      deployParachute();
//      deployed = true;
//    }
//    else if (cmd == "CMD_ABORT") {
//      deployParachute();
//      deployed = true;
//      currentState = LANDED;
//    }
//    else if (cmd == "CMD_STATUS") {
//      LoRa.beginPacket();
//      LoRa.print(millis()); LoRa.print(",");
//      LoRa.print(lastAltitude, 2); LoRa.print(",");
//      LoRa.print(flightStateToStr(currentState));
//      LoRa.endPacket();
//    }
//  }
//}


/////////////////////////////////////////////// Pre-flight

// Function to handle retry logic for sensor initialization

int initialize_sensor_with_retry(int (*sensor_init_func)(), const char* sensor_name){
  
  int attempts = 0;
  int previus_result = 1; // Assume failure initially

  while (attempts < MAX_INITIALIZATION_RETRIES) {
    Serial.print("Attempting to initialize ");
    Serial.print(sensor_name);
    Serial.print(" (Attempt ");
    Serial.print(attempts + 1);
    Serial.println(")...");

    previus_result = sensor_init_func();

    if (previus_result == 0) {
      Serial.print(sensor_name);
      Serial.println(" initialized successfully.");
      return 0;  // Success
    }

    // If the sensor fails to initialize, provide feedback and retry
    Serial.print(sensor_name);
    Serial.println(" initialization failed.");

    digitalWrite(BUZZER_PIN, HIGH);  // Turn on buzzer to indicate failure
    delay(500);  // Give a quick indication of failure
    digitalWrite(BUZZER_PIN, LOW);   // Turn off buzzer

    delay(INITIALIZATION_RETRY_DELAY);  // Wait before retrying
    attempts++;
  }

  // If we reach the maximum retries, return failure
  return 1;  // Failed after retries
}

int initialize_BMI270_BMM150(){
  if (!IMU.begin()) return 1;
  else return 0;
}

int initialize_barometer(){
  if (!BARO.begin()) return 1;
  else return 0;
}

int initialize_lora(){
  if (!LoRa.begin(LORA_FREQENCY)) return 1;
  else {
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setTxPower(LORA_SET_GAIN);
    return 0;
  }     // initialize ratio at desired MHz and sync word
}

int initialize_sd_card_module(){
  if (!SD.begin(SD_MODULE_CS_PIN)) return 1;
  else return 0;
}

int initialize_bmp280(){
  if (!BMP280.begin()) return 1;
  else {
  BMP280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                 Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                 Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                 Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                 Adafruit_BMP280::STANDBY_MS_1);  /* Standby time. */
  return 0;
  }
}


/////////////////////////////////////////////// During Flight

void checkLaunchCondition(unsigned long current_time){
  
  if (!launched){
    buzzer_chime(1, 50, 150);
    rgb_led_blink(1, 1, 50, 150);

    static unsigned long launchStart = 0;
    //static float smoothed_acc = 0;

    float current_acceleration = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
    //float smoothed_acc = 0.9 * smoothed_acc + 0.1 * current_acceleration;

    if (current_acceleration > LAUNCH_ACCEL_THRESHOLD) {

      if (launchStart == 0) launchStart = current_time;

      if ((current_time - launchStart) > LAUNCH_MIN_DURATION) {

        launched = true;
        launchTime = current_time;

        rocket_state = ASCENT;
        Serial.println("Launch detected!");
      }
    }
    else{
      launchStart = 0;
    }
  }
}

void update_barometer_data(){
  pressure = BARO.readPressure();
  temperature = BARO.readTemperature();
}


void sendTelemetry(int time){
  LoRa.beginPacket();
  LoRa.write(LORA_HEADER);
  LoRa.print(time);
  LoRa.print(",");
  LoRa.print(rocket_state);
  LoRa.print(",");
  LoRa.print(acc_x, 2);
  LoRa.print(",");
  LoRa.print(acc_y, 2);
  LoRa.print(",");
  LoRa.print(acc_z, 2);
  LoRa.print(",");
  LoRa.print(gyro_x, 2);
  LoRa.print(",");
  LoRa.print(gyro_y, 2);
  LoRa.print(",");
  LoRa.print(gyro_z, 2);
  LoRa.print(",");
  LoRa.print(pressure, 2);
  LoRa.print(",");
  LoRa.print(temperature, 2);
  LoRa.endPacket();

}


void save_data_to_sd(int post_launch_time){
  flightdata_file = SD.open(SD_ALL_DATA_FILE_NAME, FILE_WRITE);
  if (flightdata_file){
    flightdata_file.print("Time ");
    flightdata_file.print(post_launch_time);
    flightdata_file.print(",");
    flightdata_file.print(pressure);
    flightdata_file.print(",");
    flightdata_file.print(rocket_state);
    flightdata_file.print(",");
    flightdata_file.print(acc_x, 2);
    flightdata_file.print(",");
    flightdata_file.print(acc_y, 2);
    flightdata_file.print(",");
    flightdata_file.print(acc_z, 2);
    flightdata_file.print(",");
    flightdata_file.print(gyro_x, 2);
    flightdata_file.print(",");
    flightdata_file.print(gyro_y, 2);
    flightdata_file.print(",");
    flightdata_file.print(gyro_z, 2);
    flightdata_file.print(",");
    flightdata_file.print("Temperature: ");
    flightdata_file.print(temperature);
    flightdata_file.print(", Preasure: ");
    flightdata_file.println(pressure);

  }
  flightdata_file.close();


}

void update_accelerometer_data(){
  if (IMU.accelerationAvailable()) IMU.readAcceleration(acc_x, acc_y, acc_z);
}

void update_gyro_data(){  
  if (IMU.gyroscopeAvailable()) IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
}


//////////////////////////////////////////////// ASCENT

float calculateAltitudeFromPressure(float pressure, float seaLevelPressure) {
  return 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));
}

void update_height_data(){
  current_altitude = calculateAltitudeFromPressure(pressure, SEAL_LEVEL_PRESSURE);
  //current_altitude = BMP280.readAltitude();
  
}

void check_height(){
  static int apogeeCounter = 0;
  if (current_altitude > last_altitude) last_altitude = current_altitude;
  if (current_altitude < last_altitude - 1.0) {
    apogeeCounter++;
    if (apogeeCounter > 3) {
      rocket_state = APOGEE;
      Serial.println("Apogee detected");
    }
  } else {
    apogeeCounter = 0;
  }
}





//////////////////////////////////////////////// APOGEE

void deploy_parachute(int rotate, int time){  
  
  //int previus_possition = deployServo.read();
  
  deployServo.write(rotate); // the servo rotate to the "rotate angle"
  delay(time); // stays in that possition for "time" miliseconds
  
  //deployServo.write(previus_possition); // the servo returns to its previus position after some time that the user sets
  
  deployed_parachute = true; // change the global boolean of the deployment status
  rocket_state = DESCENT;
  Serial.println("Landing detected");
}




/////////////////////////////////////////////// DECENT







/////////////////////////////////////////////// LANDED













