#include "Ultrasonic.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <math.h>
#define DEVICE (0x53)    //ADXL345 device address 
#define TO_READ (6)        //
#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial nss(2, 3);
Ultrasonic ultrasonic(12,13);
char temp[25];
const float alpha = 0.5;
double X = 0;
double Y = 0;
double Z = 0;
byte buff[TO_READ] ;    //6 bytes buffer for saving data read from the device
char str[512];  
//string buffer to transform data before sending it to the serial port
int b;
long lat,lon;
 long distance; 
 const int buttonPin =5; 
 int buttonState = 0; 
void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  nss.begin(9600);
  //Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);
   pinMode(buttonPin, INPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

}

void loop()
{
  int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
  int x, y, z;
  float roll, pitch;
 
   //Ultrasonic hcsr;
   distance = ultrasonic.Ranging(CM);
    buttonState = digitalRead(buttonPin);
   
   while (nss.available())
  {
  int c = nss.read();
  if (gps.encode(c))
  {// long lat, lon;
    gps.get_position(&lat, &lon);
    //gps.Hour(&tim);
     //sprintf(temp, "%ld,%ld",lat,lon);
  }
  }

    // Serial.println(temp);


  readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345

  //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  //thus we are converting both bytes in to one int
  x = (((int)buff[1]) << 8) | buff[0];
  y = (((int)buff[3]) << 8) | buff[2];
  z = (((int)buff[5]) << 8) | buff[4];
  //using low pass filter
  X = x * alpha + (X * (1 - alpha));
  Y = y * alpha + (Y * (1 - alpha));;
  Z = z * alpha + (Z * (1 - alpha));;
  roll = abs(atan2(-X, Z) * 180 / M_PI);
  pitch = abs(atan2(Y, sqrt(X * X + Z * Z)) * 180 / M_PI);
  if (buttonState == HIGH)
  {
    Serial.println("AT+CMGF=1");
delay(1000); 
Serial.println("AT+CMGS=\"+9779802061036\"\r" );
delay(1000); 
Serial.print("dont worry we are safe");
    
delay(1000);
Serial.println((char)26); 
delay(1000);
    }
  

 else if  (roll < 25 && pitch > 40)
  {
   // GPS1();
   GPSGSM1();
   //Serial.println(temp);
Serial.println("fall");
    delay(100);
    


    Serial.end();
  }
  else if (roll > 45 && pitch > 0.5)
  {
     GPSGSM2();
   // GPS2();
    //delay(1000);
     //Serial.println(temp);
    Serial.println("collision");
    delay(100);
      Serial.end();
  }
  else
  {
     if (distance>15)
   {
    digitalWrite(7, HIGH);
   // Serial.println("push horn");
    //Serial.println(distance);
    digitalWrite(6, LOW);
    
    delay(1);
    }
    else
    {
  digitalWrite(6, HIGH);
  //Serial.print(distance);
  //Serial.println("cm");
  digitalWrite(7, LOW);
  delay(1);
    }

  delay(1);
    }
    //else if(roll>abs(5)&&pitch>abs(20))
    // {
    //Serial.print("accident");
    //}

    //else
    //we send the x y z values as a string to the serial port
    // sprintf(str, "%d %d", roll,pitch);
    // Serial.println(str);
    //Serial.print("\t");
    //Serial.print(10);
    // Serial.print("roll");
    //Serial.print(roll);
    //Serial.print(":");
    //Serial.print("pitch");
    //Serial.println(pitch);

    //It appears that delay is needed in order not to clog the port
    delay(5);

}

//---------------- Functions
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device); //start transmission to device
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[])
{
  Wire.beginTransmission(device); //start transmission to device
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission

  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device

  int i = 0;
  // float pitch;
  // float roll;
  while (Wire.available())   //device may send less than requested (abnormal)
  {
    //if(pitch<fabs(30)|roll<fabs(7))
    buff[i] = Wire.read(); // receive a byte
    i++;


  }
  Wire.endTransmission(); //end transmission
}
void GPS1()
{

   while (nss.available())
  {
  int c = nss.read();
  if (gps.encode(c))
  { long lat, lon;
    gps.get_position(&lat, &lon);
    //gps.Hour(&tim);
     sprintf(temp, "%ld,%ld",lat,lon);
     Serial.println(temp);
    //Serial.print("vehicle no 1325 meet an accident, it fell at the place having ");
    //Serial.print("Position: ");
    //Serial.print("lat: "); Serial.print(lat); Serial.print(" ");
    //Serial.print("lon: "); Serial.println(lon);
    //Serial.end();
    }
  }
}

void GPS2()

  {

   while (nss.available())
   {
  int c = nss.read();
  if (gps.encode(c))
  { long lat, lon;
    gps.get_position(&lat, &lon);
    //gps.Hour(&tim);
    // sprintf(temp, "%ld,%ld",lat,lon);
    // Serial.println(temp);
    Serial.print("vehicle no 1325 met an accident just now,  it got collision at the place having ");
    Serial.print("Position: ");
    Serial.print("lat: "); Serial.print(lat); Serial.print(" ");
    Serial.print("lon: "); Serial.println(lon);
    Serial.end();
    delay(1000);
    }
  }}
  void GPSGSM1()
{
  Serial.println("AT+CMGF=1");
delay(1000); 
Serial.println("AT+CMGS=\"+9779802061036\"\r" );
delay(1000); 
  Serial.print("vehicle no 1325 met an accident just now,  it fell at the place having ");
  Serial.print("Position: ");
  Serial.print("lat: "); Serial.print(lat); Serial.print(" ");
  Serial.print("lon: "); Serial.println(lon);
delay(1000);
Serial.println((char)26); 
delay(1000);
}
  
void GPSGSM2()

{
  Serial.println("AT+CMGF=1");
delay(1000); 
Serial.println("AT+CMGS=\"+9779802061036\"\r" );
delay(1000); 
Serial.print("vehicle no 1325 met an accident just now,  it got collision at the place having ");
    Serial.print("Position: ");
    Serial.print("lat: "); Serial.print(lat); Serial.print(" ");
    Serial.print("lon: "); Serial.println(lon);
delay(1000);
Serial.println((char)26); 
delay(100);

}
