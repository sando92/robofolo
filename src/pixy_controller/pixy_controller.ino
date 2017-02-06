#include <SPI.h>  
#include <Pixy.h>

// This is the main Pixy object 
Pixy pixy;
bool rightTurn = false;
bool leftTurn = false;
bool straightAhead = true;

void setup() {
  Serial.begin(38400); //38400 bauds needed to get all pixy's infos
  Serial.print("Starting...\n");

  pixy.init();
}

void loop() {
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  
  // grab blocks!
  blocks = pixy.getBlocks();
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    i++;
    
    // do this (print) every 50 frames because printing every
    // frame would bog down the Arduino
    if (i%50==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf); 
        pixy.blocks[j].print(); // to see object information in serial port, look at what it is exactly
        if (pixy.blocks[j].x < 130 ) {
          Serial.println("GO LEFT");
          rightTurn = false;
          leftTurn = true;
          straightAhead = false;
        } else if (pixy.blocks[j].x > 220) {
          Serial.println("GO RIGHT");
          rightTurn = true;
          leftTurn = false;
          straightAhead = false;
        } else {
          Serial.println("STRAIGHT AHEAD");
          rightTurn = false;
          leftTurn = false;
          straightAhead = true;
        }
    }
    }
  }
  
  //get pixy.blocks[i].signature, be sure with this function that the object detected is our color code.
  //hope to detect one single block, color code, and then get the pixy.blocks[i].x (y) and height, width for distance
  // I think in the first time only look at the x and y to turn as needed to follow the object.
  //we won't use the attribut angle for our use. 
    
}
