// ADS Touch Sensor Test Example Program (IC P/N:TSM16C, 28QFN)
// Code: 
// Date: 2019.01.09  Ver.: 0.0.2
// H/W Target: ARDUINO UNO R3, S/W: Arduino IDE  1.8.10
// Author: Park, Byoungbae ( yni2yni@hanmail.net )
// Note: More information? Please ,send to e-mail.
// Uno R3, A4:SDA, A5: SCL, RESET: D7, Leonardo 2:SDA,3:SCL

#include <Wire.h>

#define LF        0x0A //New Line
#define CR        0x0D //Carriage  return
#define SP        0x20 //Space

#define Sensitivity1  0x02 //ch2,ch1
#define Sensitivity2  0x03 //ch4,ch3
#define Sensitivity3  0x04 //ch6,ch5
#define Sensitivity4  0x05 //ch8,ch7
#define Sensitivity5  0x06 //ch10,ch9
#define Sensitivity6  0x07 //ch12,ch11
#define Sensitivity7  0x22 //ch14,ch13
#define Sensitivity8  0x23 //ch16,ch15

// Sensitivity levlel for Half Sensing Mode time
// In general, set value is same as normal mode sensitivity value.
#define HSensitivity1 0x31 //ch2,ch1 
#define HSensitivity2 0x32 //ch4,ch3
#define HSensitivity3 0x33 //ch6,ch5
#define HSensitivity4 0x34 //ch8,ch7
#define HSensitivity5 0x35 //ch10,ch9
#define HSensitivity6 0x36 //ch12,ch11
#define HSensitivity7 0x37 //ch14,ch13
#define HSensitivity8 0x38 //ch16,ch15

#define CTRL1     0x08  
#define CTRL2     0x09
#define Ref_rst1  0x0A
#define Ref_rst2  0x0B
#define Ch_Hold1  0x0C //Touch Key Channel Enable/Disable (Enable = 0x00)
#define Ch_Hold2 0x0D  //Touch Key Channel Enable/Disable (Enable = 0x00)
#define Cal_Hold1 0x0E //Calibration Enable/Disable (Enable = 0x00)
#define Cal_Hold2 0x0F //Calibration Enable/Disable (Enable = 0x00)

#define OUTPUT_REG1   0x10 //cs4~cs1 output
#define OUTPUT_REG2   0x11 //cs8~cs5 output
#define OUTPUT_REG3   0x12 //cs12~cs9 output
#define OUTPUT_REG4   0x13 //cs16~cs13 output
#define Lock_mask	  0x3B //Lock Mask
#define Force_en	  0x41 //Lock Mask

// =============== TSM16C I2C Chip ID ================================================
#define TSM16C_SLAVE_GND  0x68 //7bit address: 8bit address 0xD0>>1 //Chip_ID Pin = GND
#define TSM16C_SLAVE_VDD  0x78 //7bit address: 8bit address 0xF0>>1 //Chip_ID Pin = VDD

void  Init_TSM16C(void); //Initialize TSM16C (28QFN)

#define RESET_PIN 7 //Reset pin (High Pulse)

void print2hex(byte *data, byte length) //Print Hex code
{
   Serial.print("0x");
   for (int i = 0; i < length; i++)
   {
      if (data[i] < 0x10)
      {
         Serial.print("0");
      }
      Serial.print(data[i], HEX);
      Serial.write(SP);
   }
}

void setup(){
	
  delay(1); // wait for 1[msec]	
  Wire.begin();        // join i2c bus (address optional for master)
  Wire.setClock(2000000); // 200Khz
  Serial.begin(115200);  // start serial for output (Spped)
  // put your setup code here, to run once:
 
  pinMode(RESET_PIN, OUTPUT);
  //SDA, SCL = Hi-z (need to pull-up Resistor)

  // IC H/W reset signal control ,Active  High Reset
  digitalWrite(RESET_PIN, LOW); // Reset pin = low
  digitalWrite(RESET_PIN, HIGH); // Reset pin = High
  delay(2); //Min: wait for 2[msec]
  digitalWrite(RESET_PIN, LOW); // Reset pin = low
  delay(100); //wait for 100[msec]
  Init_TSM16C(); //Initialize TSM16C
  
}
void loop() {

   byte read_data[4] = {0};

   // Touch Key read
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(OUTPUT_REG1));            // sends register address (0x10h~0x13h)
   Wire.endTransmission();              // stop transmitting
   Wire.requestFrom(TSM16C_SLAVE_GND, 4); // key data read (4 byte)
   read_data[0] = Wire.read();          //Key 4~1
   read_data[1] = Wire.read();          //Key 9~5
   read_data[2] = Wire.read();          //Key 12~9
   read_data[3] = Wire.read();          //Key 16~13
   Wire.endTransmission();              // i2c end

   Serial.write(10);
   Serial.print(" Touch Sensor Output Data (Hex) ---- > "); // Test Code

   print2hex(read_data, 4);
   //Serial.write(SP);
   //Serial.write(LF);
   Serial.write(CR);

   delay(50);   
}

void  Init_TSM16C(void)
{
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL2)); // sends register address
   Wire.write(byte(0x0F)); // sends register data Software Reset Enable(Set)
   Wire.endTransmission(); // stop transmitting

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity1)); // sends register address
   Wire.write(byte(0xBB)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity2)); // sends register address
   Wire.write(byte(0xBB)); // sends register data
   Wire.endTransmission(); // stop transmitting
  
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity3)); // sends register address
   Wire.write(byte(0xBB)); // sends register data
   Wire.endTransmission(); // stop transmitting

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity4)); // sends register address
   Wire.write(byte(0xBB)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity5)); // sends register address
   Wire.write(byte(0xBB)); // sends register data
   Wire.endTransmission(); // stop transmitting   
  
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity6)); // sends register address
   Wire.write(byte(0xBB)); // sends register data
   Wire.endTransmission(); // stop transmitting   

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity7)); // sends register address
   Wire.write(byte(0xBB)); // sends register data
   Wire.endTransmission(); // stop transmitting   

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity8)); // sends register address
   Wire.write(byte(0xBB)); // sends register data
   Wire.endTransmission(); // stop transmitting

// --------------- Half Sensign Mode Sensitivity ------------------------------
// In general, set value is same as normal mode sensitivity value.
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(HSensitivity1));           // sends register address
   Wire.write(byte(0xBB));                   // sends register data
   Wire.endTransmission();                   // stop transmitting

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(HSensitivity2));           // sends register address
   Wire.write(byte(0xBB));                   // sends register data
   Wire.endTransmission();                   // stop transmitting

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(HSensitivity3));           // sends register address
   Wire.write(byte(0xBB));                   // sends register data
   Wire.endTransmission();                   // stop transmitting

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(HSensitivity4));           // sends register address
   Wire.write(byte(0xBB));                   // sends register data
   Wire.endTransmission();                   // stop transmitting

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(HSensitivity5));           // sends register address
   Wire.write(byte(0xBB));                   // sends register data
   Wire.endTransmission();                   // stop transmitting

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(HSensitivity6));           // sends register address
   Wire.write(byte(0xBB));                   // sends register data
   Wire.endTransmission();                   // stop transmitting

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(HSensitivity7));           // sends register address
   Wire.write(byte(0xBB));                   // sends register data
   Wire.endTransmission();                   // stop transmitting

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(HSensitivity8));           // sends register address
   Wire.write(byte(0xBB));                   // sends register data
   Wire.endTransmission();                   // stop transmitting
// ---------------------------------------------------------------------

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL1)); // sends register address
   Wire.write(byte(0x22)); // sends register data
   Wire.endTransmission(); // stop transmitting   
    
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Ref_rst1)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Ref_rst2)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting
  
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Ch_Hold1)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Ch_Hold2)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Cal_Hold1)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting   
   
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Cal_Hold2)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting   
 //====================================================================
 //  Single Mode setup
  /*
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL2)); // sends register address
   Wire.write(byte(0x2F)); // sends register data
   Wire.endTransmission(); // stop transmitting   

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Lock_mask)); // sends register address
   Wire.write(byte(0xA5)); // sends register data
   Wire.endTransmission(); // stop transmitting  

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Force_en)); // sends register address
   Wire.write(byte(0x94)); // sends register data
   Wire.endTransmission(); // stop transmitting  

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Lock_mask)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting  

   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL2)); // sends register address
   Wire.write(byte(0x27)); // sends register data 
   //sleep mode enable, S/W Reset Clear, Impedance High, Multi Output, Single Mode
   Wire.endTransmission(); // stop transmitting   
*/
//================================================================================
   Wire.beginTransmission(TSM16C_SLAVE_GND); // sned ic slave address
   Wire.write(byte(CTRL2)); // sends register address
   Wire.write(byte(0x07)); // sends register data , Software Reset Disable (Clear)
   //sleep mode enable, S/W Reset Clear, Impedance High, Multi Output
   Wire.endTransmission(); // stop transmitting   

   }
// End
