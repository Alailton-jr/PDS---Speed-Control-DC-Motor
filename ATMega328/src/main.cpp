

#include <Arduino.h>
// Keypad Library
#include <Keypad_I2C.h>
#include <Keypad.h>
#include <Wire.h>
#define KEYPAD_ADDR 0x20   

// LCD Library
#include <LiquidCrystal_I2C.h> 
#define LCD_col 16
#define LCD_lin 4
#define LCD_ADDR 0x38

/*
  PI Controller
  - PI_B0: Proportional gain
  - PI_B1: Derivative gain
  - PulsePerRevolution: Number of pulses per revolution of the motor
  - MAXSPD: Maximum velocity of the motor
  - MINSPD: Minimum velocity of the motor
  - Equation:
    - G(z) = (PI_B0 * ( z - PI_B1 ) ) / (z-1)
    - Voltage = Voltage_dot + PI_B0 * Error + PI_B1 * Error_dot
*/
#define PI_B0 0.4556
#define PI_B1 -0.2419
const float PulsePerRevolution = 4*11*45; // (4 INTS per encoder pulses) * (11 Encoder pulses per revolution) * (45 gearbox ratio)
#define MAX_SPD 2.0
#define MIN_SPD -2.0
#define SPD_STEP 0.2

enum Direction_E{
  Forward = 1,
  Backward = -1,
  None = 0
};

void printHome();
void configTimer1(uint16_t comp_A, uint16_t comp_B);
void configInput();

// ============ Keypad  Config ====================================
const uint8_t ROWS = 4;           // number of keypad lines
const uint8_t COLS = 4;           // number of keypad columns
uint8_t keys[ROWS][COLS] = {      // key specification
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
uint8_t rowPins[ROWS] = {4, 5, 6, 7}; //connection to the row pinouts of the keypad
uint8_t colPins[COLS] = {0, 1, 2, 3}; //connection to the column pinouts of the keypad
TwoWire *jwire = &Wire;
Keypad_I2C kpd(makeKeymap(keys), rowPins, colPins, ROWS, COLS, KEYPAD_ADDR, PCF8574, jwire);

// ========== LCD Config =======================================
LiquidCrystal_I2C lcd(LCD_ADDR,LCD_col,LCD_lin);

// ========== Global Variables ==================================
uint16_t inputCounter, timerCounter, comp_A;
float velocity, rpsConversion, PI_Error, voltage_PI, velocityRef;
Direction_E direction;

int main(){

  cli(); // disable global interrupts
  // ============ ATMega Initiate ====================================

  init(); // Call ATMega initialization functions

  //Debug
  Serial.begin(9600);
  Serial.println("Starting");

  // Initiate the peripherals
  jwire->begin(); // Initiate the I2C bus
  kpd.begin(); // Initiate the keypad
  lcd.begin(LCD_col,LCD_lin); // Initiate the LCD
  printHome(); // Print the home screen

  // Configure the ports
  PORTB = 0x00; // Reset the PORTB pins
  DDRB = (1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB5); // Set PB0, PB1, PB2 and PB5 as output

  configInput();
  configTimer1(1200, 400); // Initiate the timer 1
  

  // Initiate the variables
  rpsConversion = 1.0/PulsePerRevolution*100;
  Serial.println(rpsConversion);
  timerCounter = inputCounter = 0;
  velocityRef = velocity = 0;
  direction = None;
  char key;
  uint32_t time = 1;

  // ================================================
  sei(); // enable global interrupts

  while(1){

    key = kpd.getKey();
    if (key != NO_KEY){
      switch (key) 
      {
        case '2':
          // Turn the motor off
          lcd.setCursor(16-3,0);
          lcd.print("OFF");
          PORTB &= ~(1<<PB0); // Set PB0 to 0
          break;
        case '1':
          // Turn the motor on
          lcd.setCursor(16-3,0);
          lcd.print("ON ");
          PORTB |= (1<<PB0); // Set PB0 to 1
          break;
        case 'A':
          // Increase the reference velocity
          velocityRef += SPD_STEP;
          if (velocityRef > MAX_SPD) velocityRef = MAX_SPD;
          break;
        case 'B':
          // Decrease the reference velocity
          velocityRef -= SPD_STEP;
          if (velocityRef < MIN_SPD) velocityRef = MIN_SPD;
          break;
        default:
          break;
      }
    }

    if(millis() - time > 500){
      // Print the velocity and the reference velocity
      PORTB ^= (1<<PB5);
      time = millis();
      lcd.setCursor(16+4,0);
      lcd.print(velocity);
      lcd.setCursor(16+9,0);
      lcd.print(" | ");
      lcd.setCursor(16+12,0);
      lcd.print(velocityRef);
    }
  }

  return 0;
}

/**
 * @brief Initiliaze the LCD
*/
void printHome(){

  lcd.backlight();               // turn on the LCD backlight
  lcd.clear();                   // clears the LCD screen and positions the cursor in the upper-left corner
  lcd.setCursor(0,0);            // moves the cursor at the beginning of the first line
  // lcd.print("Control Motor   ");
  lcd.print("Motor: Power ON ");
  lcd.setCursor(0,1);            // moves the cursor at the beginning of the second line
  lcd.print("0: ON     1: OFF");
  lcd.setCursor(16,0);            // moves the cursor at the beginning of the third line
  lcd.print("RPS:            ");
  lcd.setCursor(16,1);            // moves the cursor at the beginning of the fourth line
  lcd.print("Left: A Right: B");
}

/**
 * @brief Configure the timer 1
 * @param comp_A Compare value for the output A
 * @param comp_B Compare value for the output B
*/
void configTimer1(uint16_t comp_A, uint16_t comp_B){

  /*
    Config Timer 1
    - Wave Generation Mode 10 (PWM Phase Correct)
    - Compare Output Mode 1A (Clear on Compare Match, Set at BOTTOM)
    - Compare Output Mode 1B (Clear on Compare Match, Set at BOTTOM)
    - Clock speed 16MHz
  */
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // Set the compare output mode for A and B
  TCCR1B = (1 << WGM13) | (1 << CS10); // Set the wave generation mode and the clock speed

  // Set the compare values for input A and B
  OCR1A = comp_A;  OCR1B = comp_B; 

  TIMSK1=0x01; // enable overflow interrupt
  ICR1=1600; // 16MHz/1600=10kHz
  TCNT1=0; // clear the timer counter
}

/**
 * @brief Configure the input INT0 and INT1 interrupts
*/
void configInput(){
  // Set the interrupt to trigger on the falling edge and rising edge
  EICRA = (1 << ISC10) | (1 << ISC00); 

  // Enable interrupts for INT0 and INT1
  EIMSK = (1<<INT0)|(1<<INT1);
}


// Interrupt Service Routines for INT0
ISR(INT0_vect){
  // Register the motor encoder input on the rising edge and falling edge at the input INT0

  inputCounter++;

  // INT0-> PD2 (pin 2 of the Arduino connector)
  if (PIND & (1<<PD2)){
    // If the input 2 and 3 are high the motor is moving forward
    if (PIND & (1<<PD3)) direction = Forward;
    else direction = Backward;
  }
}

// Interrupt Service Routines for INT1
ISR(INT1_vect){
  // Register the motor encoder input on the rising edge and falling edge at the input INT1
  inputCounter++;
}


// Interrupt Service Routines for Timer 1
ISR(TIMER1_OVF_vect){
  /*
    This interrupt is called every 0.1ms (10kHz) and when it's counter reaches 50,
    it computes the velocity of the motor and the PI controller output, updating the 
    compare value for the output A and B
  */

  timerCounter++;
  if (timerCounter >= 50){
    // Compute the velocity and the PI controller output
    timerCounter = 0;
    velocity = (float)inputCounter*rpsConversion*direction;
    inputCounter = 0;
    voltage_PI = voltage_PI + (PI_B0 * (velocityRef - velocity) + PI_B1 * PI_Error)*6.2831853;
    if (voltage_PI > 10.8) voltage_PI = 10.8;
    else if (voltage_PI < -10.8) voltage_PI = -10.8;
    PI_Error = (velocityRef - velocity);

    comp_A = 800 + (uint16_t)(voltage_PI*66.6666667);
    OCR1A = comp_A;
    OCR1B = 1600 - comp_A;
  }

}






