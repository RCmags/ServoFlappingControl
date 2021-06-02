// Description: Servo controller for a servo-powered ornithopter.

// Program decodes PWM signals from an RC reciever and outputs PWM signals that can be read by hobby grade servos. 
// The throttle channel is used generate a flapping motion of fixed frequency and variable amplitude, while the 
// elevator and aileron channels are used to offset the dihedral angle of the wings.

//--- Libraries:
#include <Servo.h>
#include <PinChangeInterrupt.h>

//--- Constants:  
  
  //- PWM input:    
      //Input Gains:
const float GAIN_ROLL       = -0.2;
const float GAIN_PITCH      = 0.2;
const float GAIN_AMP        = 0.5;
const float GAIN_AMP_DIFF   = 1.0; 
const float GAIN[]          = { GAIN_ROLL, GAIN_PITCH, 1.0, GAIN_AMP_DIFF };
      // Pins:
const int RC_INPUT_PINS[]   = { 9, 10, 11, 12 };
const int N_RC_INPUTS       = sizeof( RC_INPUT_PINS )/sizeof( RC_INPUT_PINS[0] );
      // Filtering:
const int RC_INPUT_DEADBAND = 4;
const int PWM_TIME_LIMIT    = 2500;
const float DECAY_INPUT     = 0.05;

  //- PWM output:
      // Restrictions:
const int PWM_CHANGE        = 1000;
const int PWM_MID           = 1500;
      // Pins:
const int PIN_LEFT          = 2;
const int PIN_RIGHT         = 3;
      // Trims:
const int TRIM_LEFT         = 0;
const int TRIM_RIGHT        = 0;

  //- Amplitude control: 
const int AMP_OFFSET        = 400;
const float REACTION_TIME   = 0.01;
const float REACTION_PARAMETER = (500.0/3.0) / REACTION_TIME ;

  //- Wave signals
const float FREQ            = 3.2;
const float SINE_CONST      = 2.0*PI/1000.0;
const float TRI_CONST       = 4.0/1000.0;

  //- Low voltage cutoff:
      // Gains:
const float VOLT_CUTOFF     = 2 * 3.4; // Voltage drops during load
const float DECAY_VOLT      = 0.001;
const float INTERVAL        = 5000.0;
      // Input pin:
const int VOLT_PIN          = A7;
const float RESISTOR_1      = 67000;
const float RESISTOR_2      = 118000;
const float VOLT_CONVERSION = ( 5.0/1024.0 )*( RESISTOR_1 + RESISTOR_2 )/RESISTOR_1; 

  //- Startup delay:
const int BLINK_NUM         = 5;
const int BLINK_TIME        = 200;

//--- Variables:
Servo servo[2];
float pwm_input[N_RC_INPUTS] = {0};
volatile int16_t pwm_raw[N_RC_INPUTS] = {0};
volatile int32_t curr_time[N_RC_INPUTS] = {0};
volatile boolean last_state[N_RC_INPUTS] = {HIGH}; // Pins default to HIGH (pullup resistor)


//--- Functions:

// -- PWN Inputs --

  //- Pin Change Interrupt to get PWM inputs: 
void checkPwmInput( void ) {
  
  for( int index = 0 ; index < N_RC_INPUTS ; index += 1 ) {
    
    if( digitalRead( RC_INPUT_PINS[index] ) == HIGH && last_state[index] == LOW ) {
      last_state[index] = HIGH;
      curr_time[index] = micros(); 
    }

    if( digitalRead( RC_INPUT_PINS[index] ) == LOW && last_state[index] == HIGH ) {

      if( curr_time[index] == 0 ) {
        curr_time[index] = micros() - PWM_MID;
      }
      
      last_state[index] = LOW;
      
      int change_time = micros() - curr_time[index] - PWM_MID;
      
      int diff = pwm_raw[index] - change_time;

      // Deadband filter:

      if( diff > RC_INPUT_DEADBAND ) {
        pwm_raw[index] = change_time - RC_INPUT_DEADBAND;
      } 
      
      if( diff < -RC_INPUT_DEADBAND) { 
        pwm_raw[index] = change_time + RC_INPUT_DEADBAND;
      }    
    }    
  }
}

  //- Function to reset PWM inputs if signals take too long [ Wires are not connected ]: 
void checkInputState( void ) {
  
  for( int index = 0 ; index < N_RC_INPUTS ; index += 1 ) {
    
    if( curr_time[index] != 0 && digitalRead( RC_INPUT_PINS[index] ) == HIGH && (micros() - curr_time[index]) > PWM_TIME_LIMIT ) {
      
      curr_time[index] = 0;
      last_state[index] = LOW;
      
      if( index == 2 ) {
        pwm_raw[2] = -AMP_OFFSET;
      } else {
        pwm_raw[index] = 0;    
      }
      
      pwm_input[index] = pwm_raw[index];
    }
  }
}

  //- Smoothing and scaling pwm inputs: 
void input_filter( void ) {
  for( int index = 0; index < N_RC_INPUTS; index += 1 ) {
    pwm_input[index] = ( pwm_raw[index]*GAIN[index] )*DECAY_INPUT + pwm_input[index]*( 1 - DECAY_INPUT );
  }
}

// -- Wave functions [for wing oscillation] --

  //- Returns sine wave with smooth transitions between frequencies:
float sin_wave( float freq ) {

  static float accum_time = 0;
  static int16_t change_time = 0;

  change_time = millis() - change_time;
  accum_time += change_time*freq;
  change_time = millis();
  
  return sin( SINE_CONST * accum_time );
}

  //- Returns triangular wave with smooth transitions between frequencies: 
float tri_wave( float freq ) {
  
  static int32_t diff_time = millis();
  static float accum = 0;
  static boolean dirr = 1;
  
  int32_t curr_time = millis();
  diff_time = curr_time - diff_time;
  
  if( dirr == true ) {
    accum += diff_time * freq*TRI_CONST;
  } else {
    accum -= diff_time * freq*TRI_CONST;
  }

  if( accum > 1 ) {
    dirr = false;
    accum = 1;
  } else if ( accum < -1 ) {
    dirr = true; 
    accum = -1;
  }

  diff_time = curr_time;
  return accum;
}

// -- Servo oscillation functions --

  //- Limits output to positive values:
int positive( int input ) {
  if( input < 0 ) {
    return 0;
  } else {
    return input;
  }
}

  //- Frequency is varied once servo cannot rotate faster: 
float frequency( float amp ) {
  if( (amp*FREQ) < REACTION_PARAMETER ) {
    return FREQ;
  } else {
    return REACTION_PARAMETER / amp;
  }
}

  //- Combines wing oscillation and dihedral angle: 
int flap_func( int amplitude, int amp_diff, int offset ) {
  if( amplitude == 0 ) {
    return constrain( offset , -PWM_CHANGE, PWM_CHANGE );
  } else {
    return constrain( offset + tri_wave( frequency(amplitude) )*positive(amplitude + amp_diff) , -PWM_CHANGE, PWM_CHANGE );
  }
}

// -- Low voltage cutoff [Soft decrease in value] --
float check_voltage( void ) {

  static float av_volt = 1024 * VOLT_CONVERSION;
  static int32_t past_time = 0;

  if( av_volt > VOLT_CUTOFF ) {    
  
    av_volt = ( analogRead( VOLT_PIN )*VOLT_CONVERSION )*DECAY_VOLT + av_volt*(1 - DECAY_VOLT);
    return 1;
  
  } else {
    
    if( past_time == 0 ) {
      past_time = millis();
    }
    
    int local_time = millis() - past_time;
    
    if( local_time < INTERVAL ) {
      return 1 - float(local_time)/INTERVAL;       
    } else {
      return 0;
    }
  } 
}

//--- Main functions:
void setup() {
  
  // Setting pin change interrupts:
  for( int index = 0; index < N_RC_INPUTS ; index += 1 ) {
    pinMode( RC_INPUT_PINS[index] , INPUT_PULLUP );
    attachPinChangeInterrupt( digitalPinToPCINT( RC_INPUT_PINS[index] ), checkPwmInput , CHANGE );
  }

  // Setting servo outputs:
  pinMode( PIN_LEFT, OUTPUT );
  servo[0].attach( PIN_LEFT );
  servo[0].writeMicroseconds( PWM_MID + TRIM_LEFT );
  
  pinMode( PIN_RIGHT, OUTPUT );
  servo[1].attach( PIN_RIGHT );
  servo[1].writeMicroseconds( PWM_MID + TRIM_RIGHT );
  
  //-

  pwm_raw[2] = -AMP_OFFSET;
  pwm_input[2] = pwm_raw[2]; 

  //-

  pinMode( VOLT_PIN , INPUT_PULLUP );

  // Startup delay:
  for( int index = 0; index < BLINK_NUM; index += 1 ) {
    digitalWrite( LED_BUILTIN, HIGH );
    delay(BLINK_TIME);
    digitalWrite( LED_BUILTIN, LOW );
    delay(BLINK_TIME);
  } 
}

void loop() {

  checkInputState();
  input_filter();

  float cutoff = check_voltage();
  int amp = positive( pwm_input[2] + AMP_OFFSET ) * cutoff;
  int diff = pwm_input[3] * cutoff;
  
  servo[0].writeMicroseconds( PWM_MID + TRIM_LEFT - flap_func( amp , diff , pwm_input[0] + pwm_input[1] ) );
  servo[1].writeMicroseconds( PWM_MID + TRIM_RIGHT + flap_func( amp , -diff , -pwm_input[0] + pwm_input[1] ) );
}
