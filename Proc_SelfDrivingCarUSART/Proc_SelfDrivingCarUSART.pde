/**
 * GettingStarted
 *
 * A sketch to list the available serial ports
 * and display characters received
 */
import processing.serial.*;
Serial myPort; // Create object from Serial class

/* Communications and sensors related variables */
int portIndex = 2; // WARNING: Set this to the port connected to XBEE Explorer (0 is the first port)
byte frameStartByte = 0x7E;
byte frameTypeTXrequest = 0x10;
int destAddressHigh = 0x13A200;
int destAddressLow = 0x40F8337E;
int val; // Data received from the serial port
float light=-9;
int rssi=100;
int counter = 0;

float min_radius=0;
float max_radius=0;

float meter_dist=-50; //dBm
float sig_expon=2.8; //progagation factor
float meter_dist1=-59.0668; //dBm
float sig_expon1=1.68; //progagation factor

int distanceLeft=100;
int distanceFront=100;

/* drawing related variables */
int sizeX=35;
int sizeY=60;
int posX=115;
int posY=20;

PFont font; // create a font for display
int fontSize = 12;
int rectX, rectY;      // Position of square button
int circleX, circleY;  // Position of circle button
int rectSize = 20;     // Diameter of rect button
int circleSize = 20;   // Diameter of circle button

//save buttons to fill
String letter;

void setup()
{
  size(800, 600); // screen size
  smooth(); // anti-aliasing for graphic display
  font = createFont("Arial.normal", fontSize);
  textFont(font);

  // Compute buttons location
  circleX = width/2+circleSize/2;
  circleY = height/2;
  rectX = width/2;
  rectY = height/2-140;
  ellipseMode(CENTER);

  println(Serial.list()); // print the list of all the ports
  println(" Connecting to -> " + Serial.list()[1]);
  myPort = new Serial(this, Serial.list()[1], 9600);
  myPort.clear(); // clear buffer
}

void draw()
{
  background(224); // draw a light gray background

  if (myPort.available() >= 23) { // Wait until we have a mouthful of data
    val=decodeRXAPIpacket(val); // try to decode the received API framlle from XBEE
    light=val;// 3300 mV of reference voltage and 10 bit resolution (1024 levels)
    println(val);
    println(light + " degrees C"); // Display message in Console8
  }
  drawbuttonComents(); // Draw Turning buttons ON/OFF buttons
  drawLightDisplay();  // Draw Display box with temperatura value
  drawDistanceDisplay();  // Draw Display box with temperatura value
}

void keyPressed() {
  // Called when a key button is pressed
  if (key == 'W' || key == 'w') {
    formatTXAPIpacket((byte) 'W'); // Turn on LED
    println("W"); // print the list of all the ports
    fill(0);
    letter="W";
    rect(rectX, rectY+rectSize*6, rectSize, rectSize); 

  } 
  else if (key == 'A' || key == 'a') {
    formatTXAPIpacket((byte) 'A'); // Tu<rn off LED
    println("A"); // print the list of all the ports
    fill(0);
    letter="A";
    rect(rectX, rectY+rectSize, rectSize, rectSize);
  }
    else if (key == 'S' || key == 's') {
    formatTXAPIpacket((byte) 'S'); // Turn on LED
    println("S"); // print the list of all the ports
    fill(0);
        letter="S";
    rect(rectX, rectY+rectSize*8, rectSize, rectSize);  
  }
    else if (key == 'D' || key == 'd') {
    formatTXAPIpacket((byte) 'D'); // Tu<rn off LED
    println("D"); // print the list of all the ports
    fill(0);
    letter="D";
    rect(rectX, rectY+rectSize*3, rectSize, rectSize);  
  }
    else if (key == 'P' || key == 'p') {
    formatTXAPIpacket((byte) 'P'); // AUTOMATIC ON
    println("P, AUTOMATIC ON"); // print the list of all the ports
    fill(0);
    letter="P";
    rect(rectX, rectY+rectSize*3, rectSize, rectSize); 
    rect(rectX, rectY+rectSize*3, rectSize, rectSize);  
    rect(rectX, rectY+rectSize*8, rectSize, rectSize);  
    rect(rectX, rectY+rectSize, rectSize, rectSize);
  }
  else if (key == 'O' || key == 'o') {
    formatTXAPIpacket((byte) 'O'); // AUTOMATIC OFF
    println("O - AUTOMATIC OFF"); // print the list of all the ports
    fill(255);
    letter="O";
    rect(rectX, rectY+rectSize*3, rectSize, rectSize); 
    rect(rectX, rectY+rectSize*3, rectSize, rectSize);  
    rect(rectX, rectY+rectSize, rectSize, rectSize);
  }
}


void drawbuttonComents() { 
  // draw buttons on screen
  fill(255);
  stroke(0);
    if(letter!=null){
        if(letter.equals("W")){
            fill(0);
            rect(rectX, rectY+rectSize*6, rectSize, rectSize); 
        }
        if( letter.equals("A")){
            fill(0);
            rect(rectX, rectY+rectSize, rectSize, rectSize);
        }
        if(letter.equals("S")){
            fill(0);
            rect(rectX, rectY+rectSize*8, rectSize, rectSize);
         }  
        if(letter.equals("D")){
            fill(0);
            rect(rectX, rectY+rectSize*3, rectSize, rectSize);  
        }
        
        if(letter.equals("P")){
            fill(0);
            rect(rectX, rectY+rectSize*3, rectSize, rectSize); 
            rect(rectX, rectY+rectSize*6, rectSize, rectSize);  
            rect(rectX, rectY+rectSize*8, rectSize, rectSize);  
            rect(rectX, rectY+rectSize, rectSize, rectSize);
        }

    }
  fill(0);
  text("Turn Left (or press 'A' or 'a')", rectX+rectSize, rectY+rectSize); 
  text("Turn Right (or press 'D' or 'd')", rectX+rectSize, rectY+rectSize*3);
  text("Turn Front (or press 'W' or 'w')", rectX+rectSize, rectY+rectSize*6); 
  text("Stop (or press 'S' or 's')", rectX+rectSize, rectY+rectSize*8);  
}


void drawLightDisplay() {
  // show light with higher numerical resolution inside a rectangle:
  fill(0);
  text("Light", width/2+15, posY+10);
  rect(width/2, posY+20, 120, 30, 7);
  fill(255);  // write text in white
  text(light + " somtehing", width/2+18, posY+40);
}


void drawDistanceDisplay() {
  // show distanceFront
  fill(0);
  text("distanceFront", width/2-150, posY+10);
  rect(width/2-165, posY+20, 120, 30, 7);
  fill(255);  // write text in white
  text(distanceFront + " cm", width/2-153, posY+40);
  
  // show distanceLeft
  fill(0);
  text("distanceLeft", width/2-150, posY+70);
  rect(width/2-165, posY+80, 120, 30, 7);
  fill(255);  // write text in white
  text(distanceLeft + " cm", width/2-153, posY+100);
  
}


void formatTXAPIpacket(byte value) {
  // Transmit key pressed using XBEE API frame
  int sum = 0; // Accumulate the checksum  

  // API frame Start Delimiter
  myPort.write(frameStartByte);

  // Length - High and low parts of the frame length (Number of bytes between the length and the checksum)
  myPort.write((byte) 0x00);
  myPort.write((byte) 0x0F);

  // Frame Type - Indicate this frame contains a Transmit Request
  myPort.write(frameTypeTXrequest); 
  sum += frameTypeTXrequest;

  // Frame ID - set to zero for no reply
  myPort.write((byte) 0x00); 
  sum += 0x00;

  // 64-bit Destination Address - The following 8 bytes indicate the 64 bit address of the recipient (high and low parts).
  // Use 0xFFFF to broadcast to all nodes.
  myPort.write((byte) ((destAddressHigh >> 24) & 0xFF));
  myPort.write((byte) ((destAddressHigh >> 16) & 0xFF));
  myPort.write((byte) ((destAddressHigh >> 8) & 0xFF));
  myPort.write((byte) ((destAddressHigh) & 0xFF));
  sum += (byte) ((destAddressHigh >> 24) & 0xFF);
  sum += (byte) ((destAddressHigh >> 16) & 0xFF);
  sum += (byte) ((destAddressHigh >> 8) & 0xFF);
  sum += (byte) (destAddressHigh & 0xFF);
  myPort.write((byte) ((destAddressLow >> 24) & 0xFF));
  myPort.write((byte) ((destAddressLow >> 16) & 0xFF));
  myPort.write((byte) ((destAddressLow >> 8) & 0xFF));
  myPort.write((byte) ((destAddressLow & 0xFF)));
  sum += ((byte) (destAddressLow >> 24) & 0xFF);
  sum += ((byte) (destAddressLow >> 16) & 0xFF);
  sum += ((byte) (destAddressLow >> 8) & 0xFF);
  sum += ((byte) (destAddressLow & 0xFF));

  // 16-bit Destination Network Address - The following 2 bytes indicate the 16-bit address of the recipient.
  // Use 0xFFFE if the address is unknown.
  myPort.write((byte)0xFF);
  myPort.write((byte)0xFE);
  sum += 0xFF+0xFE;

  // Broadcast Radius - when set to 0, the broadcast radius will be set to the maximum hops value
  myPort.write((byte)0x00);
  sum += 0x00;

  // Options
  // 0x01 - Disable retries and route repair
  // 0x20 - Enable APS encryption (if EE=1)
  // 0x40 - Use the extended transmission timeout
  myPort.write((byte)0x20);
  sum += 0x20;

  // RF Data
  myPort.write(value);
  sum += value;

  // Checksum = 0xFF - the 8 bit sum of bytes from offset 3 (Frame Type) to this byte.
  myPort.write( (byte) (0xFF - ( sum & 0xFF)));
}

int decodeRXAPIpacket(int val) {
  if (counter < 2){
    counter += 1;
    return 0;
  }
  else{
    counter = 0;
    // Function for decoding the received API frame from XBEE
    int rxvalue=0;
  
    while (myPort.read () != frameStartByte) {
      if (myPort.available()==0)
        return val; // No API frame present.
    }
  
    // The next two bytes represent the ADC measurement sent from the remote XBEE
    rxvalue = myPort.read() * 256; // add the most significant byte
    rxvalue += myPort.read(); // read the least significant byte
  
    // The next two bytes represent LEFT DISTANCE measurement sent from the remote XBEE
    distanceLeft = myPort.read() * 256; // add the most significant byte
    distanceLeft += myPort.read(); // read the least significant byte
    println("Left Dist value:"+distanceLeft);
    
    // The next two bytes represent FRONT DISTANCE measurement sent from the remote XBEE
    distanceFront = myPort.read() * 256; // add the most significant byte
    distanceFront += myPort.read(); // read the least significant byte
    println("Front Dist value:"+distanceFront);
      
    myPort.read(); // Read  the last byte (Checksum) but don't store it
    return rxvalue;
  }
}
