#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SFE_BMP180.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#define OLED_RESET 4
int lock=1;

void displaySensorDetails();
void displayDataRate();
void displayRange();
void DisplayLogo();
void eject();
double getPressure();

Adafruit_SSD1306 display(OLED_RESET);
Servo myservo;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
SFE_BMP180 pressure;
double baseline;
void displaySensorDetails()
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate()
{
  Serial.print  ("Data Rate:    "); 
  
  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 "); 
      break;
    default:
      Serial.print  ("???? "); 
      break;
  }  
  Serial.println(" Hz");  
}

void displayRange()
{
  Serial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}
void DisplayLogo()
{ display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(10,0);
    display.clearDisplay();
    display.println("Horizon");
    
    display.display();

    display.startscrollleft(0x00, 0x0F);
    delay(4000);
    display.stopscroll();
    display.clearDisplay();
  }

void eject()
 {int pos=0;
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(180);
  delay(600);
  myservo.write(2);
  delay(600);
  lock=0;
 }

void oleddisplay(float X,float Y , float Z,float EH)
{
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("X= ");display.print(X);  display.print("|EH= ");display.println(EH);

  display.print("Y= ");display.println(Y); // display.print("|AP= ");display.println(apogee);
  display.print("Z= ");display.println(Z);

  display.display();
  lock=1;
  }
double getPressure()
{
  char status;
  double T,P,p0,a;
  status = pressure.startTemperature();
  if (status != 0)
  { delay(status);
    status = pressure.getTemperature(T);
    if (status != 0)
    { status = pressure.startPressure(3);
      if (status != 0)
      { delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0)
        {return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}

void setup(){

 display.clearDisplay();
 display.display();

  Serial.begin(9600);
  Serial.println("Accelerometer Test"); Serial.println("");
  if(!accel.begin())
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  DisplayLogo();
  accel.setRange(ADXL345_RANGE_16_G);
  displaySensorDetails();
  displayDataRate();
  displayRange();
  Serial.println("");
  Serial.begin(9600);
  Serial.println("REBOOT");
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("BMP180 init fail (disconnected?)\n\n");
    while(1); // Pause forever.
  }
  baseline = getPressure();
  
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb");  
  pinMode(4, OUTPUT);
  
  
}

void loop() 
{ double currheight,P;
   
  int maxheight=2;
  float t_g=10;
  
  P = getPressure();
  
  currheight = pressure.altitude(P,baseline);
  
  double apogee= pressure.altitude(P,baseline);
  sensors_event_t event; 
  accel.getEvent(&event);
  
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  if ( event.acceleration.x<-10|| event.acceleration.y > t_g || event.acceleration.y < -t_g || event.acceleration.z > t_g || event.acceleration.z < -t_g || currheight > maxheight)
     eject();
  if (lock==0)
   { oleddisplay(event.acceleration.x,event.acceleration.y,event.acceleration.z,currheight);}
  Serial.print("relative altitude: ");
  if (currheight >= 0.0) Serial.print(" ");
  Serial.print(currheight);
  Serial.print(" meters, ");
  
  if (currheight >= 0.0) Serial.print(" ");
  Serial.print(currheight*3.28084);
  Serial.print(" feet ||  ");
  
 
}






