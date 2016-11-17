#include "DualMC33926MotorShield.h"
#include <PololuWheelEncoders.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
DualMC33926MotorShield md;
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/
void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);
  tsl.setGain(TSL2591_GAIN_MAX); // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         ");
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println("1x (Low)");
      break;
    case TSL2591_GAIN_MED:
      Serial.println("25x (Medium)");
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println("428x (High)");
      break;
    case TSL2591_GAIN_MAX:
      Serial.println("9876x (Max)");
      break;
  }
  Serial.print  ("Timing:       ");
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(" ms");
  Serial.println("------------------------------------");
  Serial.println("");
}

/**************************************************************************/
/*
    Program entry point for the Arduino sketch
*/
/**************************************************************************/
void setup(void) 
{
Serial.begin(115200);
  
  Serial.println("Starting Adafruit TSL2591 Test!");
  
  if (tsl.begin()) 
  {
    Serial.println("Found a TSL2591 sensor");
  } 
  else 
  {
    Serial.println("No sensor found ... check your wiring?");
    while (1);
  }
    
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Configure the sensor */
  configureSensor();
    
  // Now we're ready to get readings ... move on to loop()!

  Serial.println("Dual MC33926 Motor Shield");
  md.init();
  PololuWheelEncoders::init(3,5,6,11);
}
void simpleRead(void)
{
  // Simple data read example. Just read the infrared, fullspecrtrum diode 
  // or 'visible' (difference between the two) channels.
  // This can take 100-600 milliseconds! Uncomment whichever of the following you want to read
  //uint16_t x = tsl.getLuminosity(TSL2591_VISIBLE);
  uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  //uint16_t x = tsl.getLuminosity(TSL2591_INFRARED);

  Serial.print("[ "); Serial.print(millis()/1000); Serial.print(" s ] ");
  Serial.print("Luminosity: ");
  Serial.println(x, DEC);
}

/**************************************************************************/
/*
    Show how to read IR and Full Spectrum at once and convert to lux
*/
/**************************************************************************/
void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print("[ "); Serial.print(millis()); Serial.print(" ms ] ");
  Serial.print("IR: "); Serial.print(ir);  Serial.print("  ");
  Serial.print("Full: "); Serial.print(full); Serial.print("  ");
  Serial.print("Visible: "); Serial.print(full - ir); Serial.print("  ");
  Serial.print("Lux: "); Serial.println(tsl.calculateLux(full, ir));
}

/**************************************************************************/
/*
    Performs a read using the Adafruit Unified Sensor API.
*/
/**************************************************************************/
void unifiedSensorAPIRead(void)
{
  /* Get a new sensor event */ 
  sensors_event_t event;
  tsl.getEvent(&event);
 
  /* Display the results (light is measured in lux) */
  Serial.print("[ "); Serial.print(event.timestamp); Serial.print(" ms ] ");
  if ((event.light == 0) |
      (event.light > 4294966000.0) | 
      (event.light <-4294966000.0))
  {
    /* If event.light = 0 lux the sensor is probably saturated */
    /* and no reliable data could be generated! */
    /* if event.light is +/- 4294967040 there was a float over/underflow */
    Serial.println("Invalid data (adjust gain or timing)");
  }
  else
  {
    Serial.print(event.light); Serial.println(" lux");
  }
}

void loop()
{
  static float total2;
  static float total1;
  static int flag1=0;
  static int flag2=0;
  static int flag3=0;
  static int flag4=0;
  static int flag5=0;
  static int speed1;
  static int speed2;
  static int co;
  static signed int adjust=1;//adjustment to the speed everytime the loop is run
  static int counter=0;//count the total times the loop has been run
  static int lux;
  static int preLux=0;
  static long int count=0;
  static long timer1=0;//Counte until max is reached
  static long timer2=0;//Count until lux 30 is reached
  //static long testtime=27408;
  static long currentT;
  static float tagertRotate=1000;
    static int mode=0;
    static int t1=millis(); //t1 is the time it starts
  int j=PololuWheelEncoders::getCountsAndResetM2();
  int i=PololuWheelEncoders::getCountsAndResetM1();
  total2=total2+abs(j/3591.84);
  total1=total1+abs(i/3591.84);
  Serial.print(total1); Serial.print("    ");
  Serial.println(total2);

  /***************************sensor******************/
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  //Serial.print("Visible: "); Serial.print(full - ir); Serial.print("  ");
  lux=tsl.calculateLux(full, ir);
  Serial.print("Lux: "); Serial.println(tsl.calculateLux(full, ir));
  Serial.print("m: "); Serial.print(tagertRotate*3.14159265359*.09004); Serial.print(" / "); Serial.println(tagertRotate*3.14159265359*.09004*1.01);
  Serial.print("preLux: "); Serial.println(preLux);
  Serial.print("timer1: "); Serial.println(timer1);
  Serial.print("timer2: "); Serial.println(timer2);
  Serial.print("flag1: "); Serial.println(flag1);
    Serial.print("co: "); Serial.println(co);
    Serial.print("millis: "); Serial.println(millis());
  //Serial.print("total1: "); Serial.println(total1);
  
     /*******Clockreset********/
  if (preLux>(lux+5))//this is used to determine max lux
  {
    co = co+1;
  }
  else {
    co=0;
    }
  if ( preLux>lux && co>30 && (preLux-lux)>20 && flag1==0)
  {
    flag1=1;  
  }
  if ( preLux<lux && flag1 == 0)
  {
    preLux=lux; 
    timer1=millis(); 
  }
 /*********CODE 3************/
 if(((total1+total2)/2)<=tagertRotate)
 {
   if (flag4==0)
  {
    t1=millis();
    flag4=1;//First flag signals the car starts rolling 
  }//40s of neligible veering then it veers left so turn M2 down
  if(flag4==1)
  {
    if (millis()>=30000)
    {
      mode=1;//Move to phase 2 to combat increased veering
      //Serial.println("1");
    }
    if (millis()>=60000)
    {
      mode=2;//Move to phase 2 to combat increased veering
      //Serial.println("2");
    }
  }
  if(count%0==0 && mode==0)
  {
  md.setM1Speed(304);
  md.setM2Speed(330);//324 and 300 are good, but switching speed is probably better
  }
  else if(count%0!=0 && mode==0)
  {
    md.setM1Speed(306);
  md.setM2Speed(325);
  }
  if(count%0==0 && mode==1)
  {
  md.setM1Speed(304);
  md.setM2Speed(330);//324 and 300 are good, but switching speed is probably better
  }
  else if(count%0!=0 && mode==1)
  {
    md.setM1Speed(306);
  md.setM2Speed(325);
   if(count%0==0 && mode==2)
  {
  md.setM1Speed(304);
  md.setM2Speed(330);//324 and 300 are good, but switching speed is probably better
  }
  else if(count%0!=0 && mode==2)
  {
    md.setM1Speed(306);
  md.setM2Speed(325);
  }
  
  count=count+1;
  }
 }
 else
 {
  md.setM1Speed(0);
   md.setM2Speed(0);
 }
 if(flag1==1 && lux<30 &&flag3==0 && millis()>5000)//change the lux parameter
 {
  timer2=millis()-timer1;
  flag3=1;
 //tagertRotate=((testtime-998)/1464.8)/(0.09004*3.14159265359*1.01);//this equation will convert the time to the amount of rotation the car will go
                  //0.09 is the dimameter of the wheel
                  //3.14159265359*1.02 basically pi
                  //18874 is a made up coefficient 
  tagertRotate=((timer2+4130.841176)/1447.25941)/(0.09004*3.14159265359*1.01);//this is the formula used to determine the number of rotations
 //unit of rotation                  
 } 
 Serial.println("      ");
 Serial.println("this is a line");

}
