//
//    FILE: MCP23S17_digitalWrite.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: test MCP23S17 library
//     URL: https://github.com/RobTillaart/MCP23S17


#include "MCP23S17.h"


MCP23S17 MCP(10);   //  SPI CS pin 10

void reset_chip(){
  digitalWrite(8, LOW);
  delay(100);
  digitalWrite(8, HIGH);
  delay(100);
  bool b = MCP.begin(false);  // Don't setup pullups on begin
  delay(100);
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.print("MCP23S17_LIB_VERSION: ");
  Serial.println(MCP23S17_LIB_VERSION);
  delay(100);

  SPI.begin();

  // Ensure RST for the device is HIGH
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);


  reset_chip();
}

void sequentialDemo(){
  // A0, A1, A2, A3 should be set as outputs (0)
  // A5, A6, A7, A8 should be set as inputs (1)
  MCP.pinMode8(0, 0b11110000);
  
  // Cycle through each pin and set it to HIGH, wait 100ms, then set it to LOW
  for (int i = 0; i < 4; i++)
  {
    MCP.write1(i, 1);
    delay(100);

    // Read the assocated input
    int val = MCP.read1(i+4);
    if(val != 1)
    {
      Serial.println("ERROR: Pin " + String(i+4) + " did not read back as 1");
    }

    MCP.write1(i, 0);
    delay(200);
  }

  delay(1000);

}

void allOnPolarityDemo(){

  // A0, A1, A2, A3 should be set as outputs (0)
  // A5, A6, A7, A8 should be set as inputs (1)
  MCP.pinMode8(0, 0b11110000);

  // Set Input version on pin A5-A8
  // Set on all GPIO (to test that only inputs are being affected)
  MCP.setPolarity8(0, 0xFF);
  
  // Set all pins to HIGH
  MCP.write8(0, 0xFF);

  // Read back GPIO, expected to be 0x0F
  int val = MCP.read8(0);
  if(val != 0x0F)
  {
    Serial.println("ERROR: Expected pins to read back as 0x0F");
  }
  
  delay(1000);
  MCP.write8(0, 0x00);
  delay(1000);

  // Reset polarity
  MCP.setPolarity8(0, 0x00);

}

void testPullupPin(){
  
  // B0 is an INPUT_PULLUP (1)
  // B1 is an OUTPUT (0) (to drive the attached relay)
  MCP.pinMode8(1, 0b11111101);
  MCP.setPullup8(1, 0xFF);  // Ensure all pullups are enabled
  MCP.setPolarity8(1, 0b00000000);  // Ensure no polarity inversion

  MCP.write8(1, 0x0); // Ensure the relay is off

  delay(100); // The relay has some pin setup bug where it appears GND although should be floating

  // If we read B0, we should get a 1
  int val = MCP.read8(1);
  if(val != 0b11111101)
  {
    Serial.println("ERROR: Expected 0b11111101, got " + String(val, BIN));
  }

  delay(750);

  // If we write B1 to HIGH, we should see the relay turn on, and B0 should read back as 0
  MCP.write8(1, 0b00000010);
  delay(2000);
  val = MCP.read8(1);
  if(val != 0b11111110)
  {
    Serial.println("ERROR: Expected 0b11111110, got " + String(val, BIN));
  }

  MCP.write8(1, 0x0); // Ensure the relay is off
}

void loop()
{

  Serial.println("Sequential Demo");
  sequentialDemo();

  Serial.println("All On Demo w/ Polarity Test");
  allOnPolarityDemo();

  Serial.println("Test Pullup Pin");
  for(int i = 0; i < 2; i++){
    testPullupPin();
  }

  Serial.println("Resetting Chip");
  reset_chip();
  delay(2000);

}


//  -- END OF FILE --