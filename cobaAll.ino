#define MAX_BRIGHTNESS 255
#define REPORTING_PERIOD_MS 1000

#include <Adafruit_MLX90614.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <WiFi.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

MAX30105 particleSensor;
LiquidCrystal_I2C lcd(0x27,16,2);

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

//SENSOR MLX90614
float ambient_temp = 0.0;
float object_temp  = 0.0;

//SENSOR MAX30102
uint32_t irBuffer[100];
uint32_t redBuffer[100];
uint32_t tsLastReport = 0;

int32_t bufferLength;
int32_t spo2;
int32_t heartRate; 

int8_t validSPO2;
int8_t validHeartRate;
int beatAvg;
int sp02Avg;

byte pulseLED = 2; //onboard led on esp32 nodemcu
byte readLED = 19;

long lastBeat = 0;

float beatsPerMinute;
float ledBlinkFreq;

void setup() {
  //SET UP MLX90614
  Serial.begin(9600);
  Serial.println("Adafruit MLX90614 test");
  lcd.begin();
  mlx.begin();
//  timer.setInterval(300L,sensor1);
  
  //SET UP MAX30102
  ledcSetup(0, 0, 8); // PWM Channel = 0, Initial PWM Frequency = 0Hz, Resolution = 8 bits
  ledcAttachPin(pulseLED, 0); //attach pulseLED pin to PWM Channel 0
  ledcWrite(0, 255);

  Serial.print("Initializing Pulse Oximeter..");
  
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  byte ledBrightness = 50; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 69; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  Wire.setClock(100000); 
  
  Serial.println("\nSilahkan pilih pengukuran:");
  Serial.println("1. Suhu tubuh;");
  Serial.println("2. SpO2 & BPM");

  lcd.setCursor(0,0);
  lcd.print("Silahkan pilih");
  lcd.setCursor(0,1);
  lcd.print("pengukuran:");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("1. Suhu tubuh;");
  lcd.setCursor(0,1);
  lcd.print("2. SpO2 & BPM");
}

void sensor1(){
  mlx.begin();
  object_temp = mlx.readObjectTempC();
  
//    if(millis() / 1000 == 5){
//      lcd.setCursor(3,1);
//      lcd.print("       ...     ");
//    }
//    delay(500);
//  
//  
  lcd.setCursor(4,1);
  lcd.print(object_temp);
  
//  ambient_temp = mlx.readAmbientTempC();
//  Serial.print("Ambient Temp = ");
//  Serial.println(ambient_temp);
//  object_temp = mlx.readObjectTempC();
  Serial.print("Object Temp = ");
  Serial.println(object_temp);
}

void sensor2(){
  
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

//    if(irBuffer <
  }
}

void loop() {
  if (Serial.available() > 0) {
    int menu = Serial.read();
  
    switch(menu){
      case '1':
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Dekatkan tangan");
        lcd.setCursor(4,1);
        lcd.print("ke sensor");
        delay(3000);
      
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Hasil pengukuran");
        lcd.setCursor(6,1);
        for (int thisChar = 1; thisChar < 4; thisChar++) {
          lcd.print(thisChar);
          delay(500);
        }
//        delay(3000);
        lcd.setCursor(7,1);
        sensor1();
        break;
        
      case '2':
        sensor2();
        break;
    }
  }
}
