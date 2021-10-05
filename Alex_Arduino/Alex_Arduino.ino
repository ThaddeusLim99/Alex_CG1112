#include <serialize.h>
#include <math.h>
#include "packet.h"
#include "constants.h"
#include <avr/sleep.h>

#define PRR_TWI_MASK 0b10000000
#define PRR_SPI_MASK 0b00000100
#define ADCSRA_ADC_MASK 0b10000000
#define PRR_ADC_MASK 0b00000001
#define PRR_TIMER2_MASK 0b01000000
#define PRR_TIMER0_MASK 0b00100000
#define PRR_TIMER1_MASK 0b00001000
#define SMCR_SLEEP_ENABLE_MASK 0b00000001
#define SMCR_IDLE_MODE_MASK 0b11110001

//colour sensor definitions
#define S0 13
#define S1 12
#define S2 8
#define S3 7
#define sensorOut 9

typedef enum
{
    STOP = 0,
    FORWARD = 1,
    BACKWARD = 2,
    LEFT = 3,
    RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

#include <stdarg.h>


/*
   Alex's configuration constants
 */

// Number of ticks per revolution from the
// wheel encoder.
//#define PI 3.14159654
#define LEFT_COUNTS_PER_REV      181 //110
#define RIGHT_COUNTS_PER_REV     180 //74

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.42
#define ALEX_LENGTH 16
#define ALEX_BREADTH 6

float alexDiagonal = 0.0;
float alexCirc = 0.0;

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  6   // Left forward pin
#define LR                  5   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  11  // Right reverse pin

/*
   Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

//left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;


// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

volatile int colour = 0;
int analogInPin = A0; 
unsigned int SensorValue;
int trigPin = A5;
int echoPin = A4; 

//static volatile long diff = 0, diff_left = 0, diff_right = 0;

/*

   Alex Communication Routines.

 */


void WDT_off(void)
{
    /* Global interrupt should be turned OFF here if not
       already done so */
    /* Clear WDRF in MCUSR */
    MCUSR &= ~(1 << WDRF);
    /* Write logical one to WDCE and WDE */
    /* Keep old prescaler setting to prevent unintentional
       time-out */
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    /* Turn off WDT */
    WDTCSR = 0x00;
    /* Global interrupt should be turned ON here if
       subsequent operations after calling this function DO
       NOT require turning off global interrupt */
}

void setupPowerSaving() {
    // Turn off the Watchdog Timer
    WDT_off();
    // Modify PRR to shut down TWI
    PRR |= PRR_TWI_MASK;
    // Modify PRR to shut down SPI
    PRR |= PRR_SPI_MASK;
    // Modify ADCSRA to disable ADC,
    // then modify PRR to shut down ADC
    ADCSRA |= ADCSRA_ADC_MASK;
    PRR |= PRR_ADC_MASK;
    // Set the SMCR to choose the IDLE sleep mode
    SMCR |= SMCR_IDLE_MODE_MASK;
    // Do not set the Sleep Enable (SE) bit yet
    // Set Port B Pin 5 as output pin, then write a logic LOW
    DDRB |= 0b00010000;
    // to it so that the LED tied to Arduino's Pin 13 is OFF.
    PORTB &= 0b11101111; 
}

void putArduinoToIdle()
{
    // Modify PRR to shut down TIMER 0, 1, and 2
    PRR |= 0b01101000;
    // Modify SE bit in SMCR to enable (i.e., allow) sleep
    SMCR |= 0b00000001;
    // The following function puts ATmega328P’s MCU into sleep
    // it wakes up from sleep when USART serial data arrives
    sleep_cpu();
    // Modify SE bit in SMCR to disable (i.e., disallow) sleep
    SMCR &= 0b11111110;
    // Modify PRR to power up TIMER 0, 1, and 2
    PRR &= 0b10010111;
}

TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".

    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if (len == 0)
        return PACKET_INCOMPLETE;
    else
        return deserialize(buffer, len, packet);

}

void sendStatus()
{
    // Implement code to send back a packet containing key
    // information like leftTicks, rightTicks, leftRevs, rightRevs
    // forwardDist and reverseDist
    // Use the params array to store this information, and set the
    // packetType and command files accordingly, then use sendResponse
    // to send out the packet. See sendMessage on how to use sendResponse.
    TPacket statusPacket;
    statusPacket.packetType = PACKET_TYPE_RESPONSE;
    statusPacket.command = RESP_STATUS;
    //temp array
    uint32_t temp[10] = {leftForwardTicks, rightForwardTicks, leftReverseTicks, rightReverseTicks, leftForwardTicksTurns,
        rightForwardTicksTurns, leftReverseTicksTurns, rightReverseTicksTurns, forwardDist, reverseDist
    };

    memcpy(statusPacket.params, temp, sizeof(temp));
    sendResponse(&statusPacket);

}

void sendMessage(const char *message)
{
    // Sends text messages back to the Pi. Useful
    // for debugging.

    TPacket messagePacket;
    messagePacket.packetType = PACKET_TYPE_MESSAGE;
    strncpy(messagePacket.data, message, MAX_STR_LEN);
    sendResponse(&messagePacket);
}

void dbprint(char *format, ...)
{
    va_list args;
    char buffer[128];

    va_start(args, format);
    vsprintf(buffer, format, args);
    sendMessage(buffer);
}


void sendBadPacket()
{
    // Tell the Pi that it sent us a packet with a bad
    // magic number.

    TPacket badPacket;
    badPacket.packetType = PACKET_TYPE_ERROR;
    badPacket.command = RESP_BAD_PACKET;
    sendResponse(&badPacket);

}

void sendBadChecksum()
{
    // Tell the Pi that it sent us a packet with a bad
    // checksum.

    TPacket badChecksum;
    badChecksum.packetType = PACKET_TYPE_ERROR;
    badChecksum.command = RESP_BAD_CHECKSUM;
    sendResponse(&badChecksum);
}

void sendBadCommand()
{
    // Tell the Pi that we don't understand its
    // command sent to us.

    TPacket badCommand;
    badCommand.packetType = PACKET_TYPE_ERROR;
    badCommand.command = RESP_BAD_COMMAND;
    sendResponse(&badCommand);

}

void sendBadResponse()
{
    TPacket badResponse;
    badResponse.packetType = PACKET_TYPE_ERROR;
    badResponse.command = RESP_BAD_RESPONSE;
    sendResponse(&badResponse);
}

void sendOK()
{
    TPacket okPacket;
    okPacket.packetType = PACKET_TYPE_RESPONSE;
    okPacket.command = RESP_OK;
    sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
    // Takes a packet, serializes it then sends it out
    // over the serial port.
    char buffer[PACKET_SIZE];
    int len;

    len = serialize(buffer, packet, sizeof(TPacket));
    writeSerial(buffer, len);
}


void setupColourSensor() {
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);

    // Setting the sensorOut as an input
    pinMode(sensorOut, INPUT);

    // Setting frequency scaling to 20%
    digitalWrite(S0,HIGH);
    digitalWrite(S1,LOW);
}

int getColour() {
    // Stores frequency read by the photodiodes
    int redFrequency = 0;
    int greenFrequency = 0;
    int blueFrequency = 0;

    // Stores the red. green and blue colors
    int redColor = 0;
    int greenColor = 0;
    int blueColor = 0;

    digitalWrite(S2,LOW);
    digitalWrite(S3,LOW);

    redFrequency = pulseIn(sensorOut, LOW);
    // Remaping the value of the RED (R) frequency from 0 to 255
    redColor = map(redFrequency, 30, 120, 255, 0);
    
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);

    // Reading the output frequency
    greenFrequency = pulseIn(sensorOut, LOW);
    // Remaping the value of the GREEN (G) frequency from 0 to 255
    greenColor = map(greenFrequency, 120, 199, 255, 0);


//    digitalWrite(S2,LOW);
//    digitalWrite(S3,HIGH);
////
////    // Reading the output frequency
//    blueFrequency = pulseIn(sensorOut, LOW);
////    // Remaping the value of the BLUE (B) frequency from 0 to 255
//    blueColor = map(blueFrequency, 38, 84, 255, 0);

    if (redColor > greenColor) {// && redColor > blueColor) {
        return 1;
    }
    if (greenColor > redColor) {// && greenColor > blueColor) {
        return 2;
    }
    return 0;
}

void sendColour()
{
    colour = getColour();
    if (colour == 1) {
        sendMessage("red");
    }
    else if (colour == 2) {
        sendMessage("green");
    }
    else {
        sendMessage("error");
    }
}

void setupIRSensor()
{
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
//    pinMode(trigPin, OUTPUT);
//    pinMode(echoPin, INPUT);
    
}

int getIR()
{
    int SensorValue = digitalRead(A1);
    return SensorValue;
}

int getIR2()
{
    int SensorValue = digitalRead(A2);
    return SensorValue;
}

bool ultrasonicDist() {
    digitalWrite(trigPin, LOW);
      //delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    //delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    float duration = pulseIn(echoPin, HIGH);
    float distance = (duration*.0343)/2;
    if (distance < 4.00) {
      return true;
    }
    return false;
}
/*
   Setup and start codes for external interrupts and
   pullup resistors.

 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
    // Use bare-metal to enable the pull-up resistors on pins
    // 2 and 3. These are pins PD2 and PD3 respectively.
    // W,e set bits 2 and 3 in DDRD to 0 to make them inputs.
    DDRD = B00000000;
    PORTD |= B00001100;


}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
    if (dir == FORWARD) {
        leftForwardTicks++;
      //  forwardDist = (unsigned long) ((float) leftForwardTicks / LEFT_COUNTS_PER_REV * WHEEL_CIRC);
       // diff = leftForwardTicks - rightForwardTicks;
    } else if (dir == BACKWARD) {
        leftReverseTicks++;
    //    reverseDist = (unsigned long) ((float) leftReverseTicks / LEFT_COUNTS_PER_REV * WHEEL_CIRC);
      //  diff = leftReverseTicks - rightReverseTicks;
    } else if (dir == LEFT) {
        leftReverseTicksTurns++;
      //  diff = leftReverseTicksTurns - rightForwardTicksTurns;
    } else if (dir == RIGHT) {
        leftForwardTicksTurns++;
     //   diff = leftForwardTicksTurns - rightReverseTicksTurns;
    }
    //diff_left = (diff > 0) ? diff : -diff;
   // diff_right = (diff > 0) ? -diff : diff;
}

void rightISR()
{
    if (dir == FORWARD) {
        rightForwardTicks++;
        forwardDist = (unsigned long) (((float) rightForwardTicks / (1.5*RIGHT_COUNTS_PER_REV)) * WHEEL_CIRC);
    } else if (dir == BACKWARD) {
        rightReverseTicks++;
        reverseDist = (unsigned long) ((float) rightReverseTicks / RIGHT_COUNTS_PER_REV * WHEEL_CIRC);
    } else if (dir == LEFT) {
        rightForwardTicksTurns++;
    } else if (dir == RIGHT) {
        rightReverseTicksTurns++;
    }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
    // Use bare-metal to configure pins 2 and 3 to be
    // falling edge triggered. Remember to enable
    // the INT0 and INT1 interrupts.
    EICRA |= 0b00001010;
    EIMSK |= 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect)
{
    leftISR();
}

ISR(INT1_vect)
{
    rightISR();
}

// Implement INT0 and INT1 ISRs above.

/*
   Setup and start codes for serial communications

 */
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
    // To replace later with bare-metal.
    Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
    // Empty for now. To be replaced with bare-metal code
    // later on.

}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

    int count = 0;

    while (Serial.available())
        buffer[count++] = Serial.read();

    return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
    Serial.write(buffer, len);
}

/*
   Alex's motor drivers.

 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.

void setupMotors()
{
    /* Our motor set up is:
       A1IN - Pin 5, PD5, OC0B
       A2IN - Pin 6, PD6, OC0A
       B1IN - Pin 10, PB2, OC1B
       B2In - pIN 11, PB3, OC2A
     */
   /* 
       DDRD |= 0b01100000; 
       DDRB |= 0b00000110;

       TCNT0 = 0;
       TIMSK0 |= 0b110;
       OCR0A = 0;
       OCR0B = 0;
    //TCCR0A = 101000001;
    TCCR0B = 000000001;
    //controls pin6 and 5

    TCNT1 = 0;
    OCR1AL = 0;
    OCR1AH  = 0;
    OCR1BL = 0;
    OCR1BH = 0;
    TIMSK1 |= 0b110;
    TCCR1A = 00100001;
    TCCR1B = 00000001;
    //controls pin 10

    TCNT2 = 0;
    OCR2A = 0;
    TIMSK2 |= 0b110;
    TCCR2A = 1000001;
    TCCR2B = 0000001;
*/     
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{

}
/*
void analog_Write(int pin,int val){
   int val_scaled = (int) ((val/100.0) * 255);
   if(pin == LF){ //PD6
   TCCR0A = 10000001;
   OCR0A = val_scaled;  
   }else if(pin == LR){ //PD5
   TCCR0A = 00100001;
   OCR0B = val_scaled;
   }else if(pin == RF){ //PB2
   TCCR1A = 10000001;
   OCR1BL = val_scaled;
   OCR1BH = 0; 
   }else if(pin == RR){ //PB1
//TCCR1A = 00100001;
    OCR2A = val_scaled;
//OCR1AH = 0;
}


switch(pin){
case LF: //PD6
OCR1
case LR: //PD5

case RF: //PB2

case RR: //PB1
}

} */
 

// Convert percentages to PWM values
int pwmVal(float speed)
{
    if (speed < 0.0)
        speed = 0;

    if (speed > 100.0)
        speed = 100.0;

    return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
    dir = FORWARD;

    if (dist > 0) {
        deltaDist = dist;
    } else {
        deltaDist = 9999999;
    }

    newDist = forwardDist + deltaDist;

    int rightval = (speed == 100) ? pwmVal(speed) : pwmVal(speed*0.8); //0.8
    int leftval = pwmVal(speed);

    // For now we will ignore dist and move
    // forward indefinitely. We will fix this
    // in Week 9.

    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.
    //while(forwardDist < newDist){
    //diff_left = diff > 0? diff : 0;
    //diff_right = diff > 0? 0 : diff;
    analogWrite(LF, leftval );
    analogWrite(RF, rightval);
    analogWrite(LR, 0);
    analogWrite(RR, 0);
    //}
   // deltaDist = 0;
    //newDist = 0;
    //stop();
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
    dir = BACKWARD;

    if (dist > 0) {
        deltaDist = dist;
    } else {
        deltaDist = 9999999;
    }

    newDist = reverseDist + deltaDist;
    int rightval = (speed == 100) ? pwmVal(speed) : pwmVal(speed*0.8); //0.8
    int leftval = pwmVal(speed);


    // For now we will ignore dist and
    // reverse indefinitely. We will fix this
    // in Week 9.

    // LF = Left forward pin, LR = Left reverse pin
    // RF = Right forward pin, RR = Right reverse pin
    // This will be replaced later with bare-metal code.
    analogWrite(LR, leftval);
    analogWrite(RR, rightval);
    analogWrite(LF, 0);
    analogWrite(RF, 0);
}

unsigned long computeDeltaTicks(float ang){
  unsigned long ticks = (unsigned long) ((ang * alexCirc * RIGHT_COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed) //change float ang to int ang
{
    float temp = ang;
    dir = LEFT;
    int val = pwmVal(speed);

    if(ang = 0)
      deltaTicks = 9999999;
    else
    {
      deltaTicks = computeDeltaTicks(ang);
    }
    targetTicks = rightForwardTicksTurns + floor(temp); //leftreverse 10
    //Serial.println(targetTicks);
    // For now we will ignore ang. We will fix this in Week 9.
    // We will also replace this code with bare-metal later.
    // To turn left we reverse the left wheel and move
    // the right wheel forward.
    analogWrite(LR, val);
    analogWrite(RF, val);
    analogWrite(LF, 0);
    analogWrite(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)    
{
  float temp = ang;
    dir = RIGHT;
    int val = pwmVal(speed);
    if(ang = 0)
      deltaTicks = 9999999;
    else
    {
      deltaTicks = computeDeltaTicks(ang);
    }
    targetTicks = rightReverseTicksTurns + floor(temp); //10
    // For now we will ignore ang. We will fix this in Week 9.
    // We will also replace this code with bare-metal later.
    // To turn right we reverse the right wheel and move
    // the left wheel forward.
    analogWrite(RR, val);
    analogWrite(LF, val);
    analogWrite(LR, 0);
    analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
    analogWrite(LF, 0);
    analogWrite(LR, 0);
    analogWrite(RF, 0);
    analogWrite(RR, 0);
    dir = STOP;
}

/*
   Alex's setup and run codes

 */

// Clears all our counters
void clearCounters()
{
    leftForwardTicks = 0;
    rightForwardTicks = 0;
    leftReverseTicks = 0;
    rightReverseTicks = 0;
    leftForwardTicksTurns = 0;
    rightForwardTicksTurns = 0;
    leftReverseTicksTurns = 0;
    rightReverseTicksTurns = 0;
    leftRevs = 0;
    rightRevs = 0;
    forwardDist = 0;
    reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
    clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
    clearCounters();
}

void handleCommand(TPacket *command)
{
    switch (command->command)
    {
        // For movement commands, param[0] = distance, param[1] = speed.
        case COMMAND_FORWARD:
            sendOK();
            forward((float) command->params[0], (float) command->params[1]);
            break;
        case COMMAND_REVERSE:
            sendOK();
            reverse((float) command->params[0], (float) command->params[1]);
            break;
        case COMMAND_TURN_LEFT:
            sendOK();
            left((float) command->params[0], (float) command->params[1]);
            break;
        case COMMAND_TURN_RIGHT:
            sendOK();
            right((float) command->params[0], (float) command->params[1]);
            break;
        case COMMAND_STOP:
            sendOK();
            stop();
            break;
        case COMMAND_GET_STATS:
            sendOK();
            sendStatus();
            break;
        case COMMAND_CLEAR_STATS:
            sendOK();
            clearOneCounter(command->params[0]);
            break;
            //new command 7
        case COMMAND_GET_COLOUR:
            sendOK();
            sendColour();
            break;
        default:
            sendBadCommand();
    }
}

void waitForHello()
{
    int exit = 0;

    while (!exit)
    {
        TPacket hello;
        TResult result;

        do
        {
            result = readPacket(&hello);
        } while (result == PACKET_INCOMPLETE);

        if (result == PACKET_OK)
        {
            if (hello.packetType == PACKET_TYPE_HELLO)
            {


                sendOK();
                exit = 1;
            }
            else
                sendBadResponse();
        }
        else if (result == PACKET_BAD)
        {
            sendBadPacket();
        }
        else if (result == PACKET_CHECKSUM_BAD)
            sendBadChecksum();
    } // !exit
}

void setup() {
    // put your setup code here, to run once:

    alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
    alexCirc = PI * alexDiagonal;

    cli();
    setupPowerSaving();
    setupEINT();
    setupSerial();
    startSerial();
    setupMotors();
    startMotors();
    enablePullups();
    initializeState();
    setupColourSensor(); //added
    setupIRSensor(); //added
    sei();
}

void handlePacket(TPacket *packet)
{
    switch (packet->packetType)
    {
        case PACKET_TYPE_COMMAND:
            handleCommand(packet);
            break;

        case PACKET_TYPE_RESPONSE:
            break;

        case PACKET_TYPE_ERROR:
            break;

        case PACKET_TYPE_MESSAGE:
            break;

        case PACKET_TYPE_HELLO:
            break;

    }
}

void loop() {
    //dbprint("PI is %3.2f\n", PI);

    // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2
    int IR1 = getIR();
    int IR2 = getIR2();
   
    if(dir == STOP) {
        putArduinoToIdle();
    }
    else if (dir != BACKWARD && IR1 == 0) {
            stop();
            sendMessage("stop");
    }
    else if (dir != BACKWARD && IR2 == 0) {
      stop();
      sendMessage("stopU");
    }
    //forward(0,100);

    //right(0,100);

    // /reverse(0,100);

    // Uncomment the code below for Week 9 Studio 2


    //  your main code here, to run repeatedly:
    TPacket recvPacket; // This holds commands from the Pi

    TResult result = readPacket(&recvPacket);

    if (result == PACKET_OK)
        handlePacket(&recvPacket);
    else if (result == PACKET_BAD)
    {
        sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
    {
        sendBadChecksum();
    }
  
    if (deltaDist > 0)
    {
        if (dir == FORWARD)
        {
            if (forwardDist > newDist)
            {
                deltaDist = 0;
                newDist = 0;
                stop();
            }
        }
        else if (dir == BACKWARD)
        {
            if (reverseDist > newDist)
            {
                deltaDist = 0;
                newDist = 0;
                stop();
            }
        }
        else if (dir == STOP)
        {
            deltaDist = 0;
            newDist = 0;
            stop();
        }
    }
    
  //  if (deltaTicks > 0){
      if(dir == LEFT){
        if(rightForwardTicksTurns >= targetTicks){
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
      }else if(dir == RIGHT){
        if(rightReverseTicksTurns >= targetTicks){
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
      }else if(dir == STOP){
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
  //  }


}
