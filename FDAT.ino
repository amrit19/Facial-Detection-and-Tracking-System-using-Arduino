////////////////////////////Arduino Code/////////////////
#include <Servo.h>

#define  servomaxx   180   // maximum degree servo horizontal (x) can turn
#define  servomaxy   180   // maximum degree servo vertical (y) can turn
#define  screenmaxx   320   // maximum screen horizontal (x) resolution
#define  screenmaxy   240   // maximum screen vertical (y) resolution
#define  servocenterx   90  // center of x servo
#define  servocentery   90  // center of y servo
#define  servopinx   9     // digital pin for servo x
#define  servopiny   10    // digital pin for servo y
#define  baudrate 115200    // communication port speed
#define distancex 1        // x servo rotation steps
#define distancey 2        // y servo rotation steps

int valx = 0;       // store x data from serial port
int valy = 0;       // store y data from serial port
int posx = 0;       // current position of x servo
int posy = 0;       // current position of y servo
int incx = 10;      // significant increments of horizontal (x) camera movement
int incy = 10;      // significant increments of vertical (y) camera movement

Servo servox;
Servo servoy;

short MSB = 0;      // to build 2-byte integer from serial input byte
short LSB = 0;      // to build 2-byte integer from serial input byte
int   MSBLSB = 0;   // to build 2-byte integer from serial input byte

void setup() {
  Serial.begin(baudrate);        // connect to the serial port
  Serial.println("Starting Cam-Servo Face tracker");

  pinMode(servopinx, OUTPUT);    // declare the servo x pin as output
  pinMode(servopiny, OUTPUT);    // declare the servo y pin as output

  servoy.attach(servopiny); 
  servox.attach(servopinx); 

  // center servos
  servox.write(servocenterx); 
  delay(200);
  servoy.write(servocentery); 
  delay(200);
}

void loop () {
  while(Serial.available() <= 0); // wait for incoming serial data
  if (Serial.available() >= 4)   // wait for 4 bytes
  {
    // get X axis 2-byte integer from serial
    //MSB = Serial.read();
    //delay(5);
    LSB = Serial.read();
    //MSBLSB = word(MSB, LSB);
    valx = int(LSB); 
    delay(5);

    // get Y axis 2-byte integer from serial
    //MSB = Serial.read();
    //delay(5);
    LSB = Serial.read();
    //MSBLSB = word(MSB, LSB);
    valy = int(LSB); 
    delay(5);

    // read last servos positions
    posx = servox.read(); 
    posy = servoy.read();

    // Find out if the X component of the face is to the left of the middle of the screen.
    if(valx < (screenmaxy/2 - incx)){
      if( posx >= 5/*incx*/ ) posx += distancex; // Update the pan position variable to move the servo to the left.
    }
    // Find out if the X component of the face is to the right of the middle of the screen.
    else if(valx > screenmaxy/2 + incx){
      if(posx <= 175/*servomaxx-incx*/) posx -= distancex; // Update the pan position variable to move the servo to the right.
    }

    // Find out if the Y component of the face is below the middle of the screen.
    if(valy < (screenmaxx/2 - incy)){
      if(posy >= 5) posy -= distancey; // If it is below the middle of the screen, update the tilt position variable to lower the tilt servo.
    }
    // Find out if the Y component of the face is above the middle of the screen.
    else if(valy > (screenmaxx/2 + incy)){
      if(posy <= 175) posy += distancey; // Update the tilt position variable to raise the tilt servo.
    }

    // Servos will rotate accordingly 
    servox.write(posx);
    servoy.write(posy);
  }   
}
