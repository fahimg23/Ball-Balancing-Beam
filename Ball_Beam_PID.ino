unsigned char * regTimer = (unsigned char*)0xB2; // timer counter reg 2
unsigned short* ADCreg = (unsigned short*)0x78;
unsigned short ADCval;

void setup() {
  
  //setting up input for ultrasonic, using timer 2 as necessary
  TCCR2A = 0b00000000;
  TCCR2B = 0b00000010; // make clock frequency 2 MHz, this means 20 ticks will take 10 us.
  OCR2A = 20; // used to for compare match flag and 20 ticks = 10 us
  DDRD = 0b00001000; // pin 6 to trigger (output), pin 7 to echo (input), ultrasonic 1 
  DDRB = 0b00000001; //pin 8 to trigger (output), pin 9 to echo (input), ultrasonic 2

  //setting up pwm for servo
  DDRD |= 0b00100000; //output pwm on pin 5
  //timer 0 control register A, fast pwm, top = OCR0A, clear on compare match, freq = 245 Hz (period =  4.08 ms), PS = 256, output is on 0C2B (comob needs to be set)
  TCCR0A = 0b00100011;
  TCCR0B = 0b00001100;
  OCR0A = 255;
  
  //initialize the servo to keep the beam in the balanced state
  OCR0B = 65;

  //setting up analog input for potentiometer (for PID tuning), not used as of now
  DDRC = 0x00; 
  DIDR0 = 0x01; // disable digital input on pin A0
  ADMUX = 0b01000000; // ref = vcc = 5v, adlar = 0
  ADCSRA = 0b10100111;
  ADCSRB = 0x00;
  ADCSRA |= 0x40; // start conversion
  Serial.begin(9600);
}

float objDist1;
float objDist2;
int timeout;
int counter1;
int counter2;
int outOfRangeFlag1;
int outOfRangeFlag2;

// sensor 1
int getDistance1(){
  
  //clearing all timer 0 flags;
  TIFR2 = 0x07;

  //Reading the Ultrasonic Sensor on pin 4
  PORTD &= 0b11110111; // clear pin 3
  //first wait 10 us so to not interfere with the previous completion of distance reading
  *regTimer = 0;
  while((TIFR2 & 0x02) != 0x02){} //breaks out of loop if compare match occured, 10 us passed
  TIFR2 |= 0x02; // clear the output compare match A flag

  // do some initializations before entering time critical area of code
  outOfRangeFlag1 = 0;
  counter1 = 0;
  //set pin 3 (trigger) high for 10 us (wait for 20 clock ticks)
  PORTD |= 0b00001000;
  while((TIFR2 & 0x02) != 0x02){} //breaks out of loop if compare match occured, 10 us passed, set trigger low to prepare for echo signal
  PORTD &= 0b11110111;
  
  //wait until echo begins, if no echo signal occurs then break after certain amount of time
  while((PIND & 0x10) != 0x10){
    outOfRangeFlag1++;
    if (outOfRangeFlag1 > 10100){
      return 0; // once again out of range
    }
  }
  // explanation of the 10100 in the loop above and below:
  // if object is 200 cm away, the pulse width time will be 200*58 us = 11600 us. Each loop execution takes about 2.3 ticks at 2 Mhz, which is 1.15 us. So, total number of loop executions
  // to get to 11600 us is 11600 us/ 1.15 us = about 10100, which is the number of loop executions at which the object is what we'll consider beyond range (beyond about 2 m). 
  while((PIND & 0x10) == 0x10){
    counter1++;
    if (counter1 > 10100){
      return 0; // once again out of range
    }
  } // this loop has been determined to take 14 clock ticks (using the timer 0 setup in this code which is at 2 Mhz), for the counter value to count up to 6, thus each execution of loop takes about 2.3 ticks.
  
  //breaks out of loop if pulse has gone LOW (ultrasonic has finished reading range), or object distance out of range

  // taking into account the current tick of the timer register, totalticks = *regTimer + 2.3 * counter.
  // each tick in this setup for the timer takes 0.5 us, totalticks *0.5 = the pulse width in us. dividng by 58 is the formula to get the distance.
  // the - 2.0 is experimentally found to give more accurate reading.
  // 0.5/58 =~ 0.009 = ~0.01
  objDist1 = (((float)(*regTimer) + (2.2 * (float)counter1)) * 0.5/58) - 2.0; 
  //objDist1 = ((float)(*regTimer + 2*counter1) * 0.01) - 2;
  return objDist1;
}

//sensor 2
int getDistance2(){
  PORTB &= 0b11111110; // clear pin 8
  //first wait 10 us so to not interfere with the previous completion of distance reading
  *regTimer = 0;
  while((TIFR2 & 0x02) != 0x02){} //breaks out of loop if compare match occured, 10 us passed
  TIFR2 |= 0x02; // clear the output compare match A flag

  // do some initializations before entering time critical area of code
  outOfRangeFlag2 = 0;
  counter2 = 0;
  //set pin 8 (trigger) high for 10 us (wait for 20 clock ticks)
  PORTB |= 0b00000001;
  while((TIFR2 & 0x02) != 0x02){} //breaks out of loop if compare match occured, 10 us passed, set trigger low to prepare for echo signal
  PORTB &= 0b11111110;
  
  //wait until echo begins, if no echo signal occurs then break after certain amount of time
  while((PINB & 0x02) != 0x02){
    outOfRangeFlag2++;
    if (outOfRangeFlag2 > 10100){
      return 0; // once again out of range
    }
  }
  // explanation of the 7600 in the loop above and below:
  // if object is 200 cm away, the pulse width time will be 200*58 us = 11600 us. Each loop execution takes about 2.3 ticks at 2 Mhz, which is 1.15 us. So, total number of loop executions
  // to get to 11600 us is 11600 us/ 1.15 us = about 10100, which is the number of loop executions at which the object is what we'll consider beyond range (beyond about 2 m). 
  while((PINB & 0x02) == 0x02){
    counter2++;
    if (counter2 > 10100){
      return 0; // once again out of range
    }
  } // this loop has been determined to take 14 clock ticks (using the timer 0 setup in this code which is at 2 Mhz), for the counter value to count up to 6, thus each execution of loop takes about 2.3 ticks.
  
  //breaks out of loop if pulse has gone LOW (ultrasonic has finished reading range), or object distance out of range

  // taking into account the current tick of the timer register, totalticks = *regTimer + 2.3 * counter.
  // each tick in this setup for the timer takes 0.5 us, totalticks *0.5 = the pulse width in us. dividng by 58 is the formula to get the distance.
  // the - 2.0 is experimentally found to give more accurate reading.
  // 0.5/58 =~ 0.009 = ~0.01
  objDist2 = (((float)(*regTimer) + (2.2 * (float)counter2)) * 0.5/58) - 2.0; 
  //objDist2 = ((float)(*regTimer + 2*counter2) * 0.01) - 2;
  return objDist2;
}

int distance1 = 0;
int distance2 = 0;
int cushion = 30; // the amount of difference to be allowed between consecutive ultrasonic readings (this will change depending on max velocity of ball in the system and how fast the ultrasonic is sampling distances)
int prevDist1 = 0;
int prevDist2 = 0;
bool initialDistFlag1 = true;
bool initialDistFlag2 = true; 

//Proportional of PID
int setPoint = 55; // desired distance from ultrasonic to ball (this is the middle of the beam)

int error1;
float errorMath1;
float Kp1 = -0.8; //proportional factor

int error2;
float errorMath2;
float Kp2 = 0.9;

//Derivative of PID
float dt = 0.8;

float velocityMath1; // velocity = (current distance - previous distance)/(10 * 5 ms), the 10 is to convert centimeters to millimeters
float Kd1 = 3.0; // derivative factor

float velocityMath2;
float Kd2 = -3.0;

int balancedPos = 66; //  OCR0B = 66, servo angle that balances the beam (straigtens it)

void loop(){
  
  distance1 = getDistance1();
  delay(10);
  distance2 = getDistance2();
  
  if(distance1 > 0 && distance1 < 53){ //filters out fluctuating values which occur after 50 cm
    //at first run of this distance measurement the prevDist variable must be set to the current distance reading
    // uses getDist and prevDist to filter out consecutive values that are too far apart from each other (in this case 5 cm) indicating inconsistent readings
    // ex. if prev reading was 25, then the current readings should be between 20 and 30 (20 to 25 if the ball is rolling towards sensor, or 25 to 30 if ball rolling away from sensor, but must be in the range from 20 to 30 hence the '&&')
    if((distance1 <= prevDist1 + cushion) && (distance1 >= prevDist1 - cushion)){
      //error1 = setPoint - distance1;
      error1 = ((float)setPoint - (float)distance1)*Kp1;
      velocityMath1 = ((float)distance1 - (float)prevDist1)/dt;
      OCR0B = (int)(balancedPos + error1 + velocityMath1*Kd1);
      prevDist1 = distance1;
    }
  }
  
  else if(distance2 > 0 && distance2 < 51){
    if((distance2 <= prevDist2 + cushion) && (distance2 >= prevDist2 - cushion)){
      //error2 = setPoint - distance2;
      error2 = ((float)setPoint - (float)distance2)*Kp2;
      velocityMath2 = ((float)distance2 - (float)prevDist2)/dt;
      OCR0B = (int)(balancedPos + error2 + velocityMath2*Kd2);
      prevDist2 = distance2;
    }
  }
  
  else{
    OCR0B = balancedPos;
  }
  
  delay(10);
}
