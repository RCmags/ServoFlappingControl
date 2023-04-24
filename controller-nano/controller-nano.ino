/* ORNITHOPTER SERVO CONTROLLER */

/* Sketch for an arduino NANO that operates the servos of a radio controlled ornithopter. 
 * It commands two servos to flap the wings and (if desired) two servos to control a V-tail.
 * Author: RCmags https://github.com/RCmags
*/

//=============== Connections ================
// See included schematic
  // Inputs:
// Pin 9  -> Receiver CH1
// Pin 10 -> Receiver CH2
// Pin 11 -> Receiver CH3
// Pin 12 -> Receiver CH4
// Pin A7 -> Middle of voltage divider
  // Outputs:
// Pin 2  -> Left  wing servo
// Pin 3  -> Right wing servo 
// Pin 4  -> Left  tail servo 
// Pin 5  -> Right tail servo 

//=================== Code ===================
#include <Servo.h>
#include "parameters.cpp"

//---- global variables 
Servo servo[4]; 
volatile uint16_t pwm_input[4] = {0};

//----- Input signals
// PORTB = {8 .. 13} -> using pins {9 .. 12} = B00011110

/* port change interrupt to read PWM inputs from receiver */
ISR( PCINT0_vect ) {
  static uint32_t initial_time[4] = {0}; 
  static uint8_t port_last = PINB;  
  
  // port changes
  uint32_t current_time = micros(); 
  uint8_t port_rise = ~port_last & PINB;
  uint8_t port_fall = port_last & ~PINB;
  
  // find changing pins
  for( uint8_t index = 0; index < 4; index += 1) {
    uint8_t mask = B00000010 << index;  // Start at PCINT1   
    if( port_rise & mask ) {                
        initial_time[index] = current_time;
    } else if ( port_fall & mask ) {       
        pwm_input[index] = current_time - initial_time[index];
    }
  }
  port_last = PINB;    
}

void setupISR() {
  // enable PORTB interrupts  
  PCICR |= (1 << PCIE0);
  // enable PCINT for pins 9-12                                                   
  PCMSK0 |= (1 << PCINT1);                                              
  PCMSK0 |= (1 << PCINT2);                                              
  PCMSK0 |= (1 << PCINT3); 
  PCMSK0 |= (1 << PCINT4); 
  // set pins inputs
  pinMode(9,  INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
}

//----- Input filter

/* convert PWM inputs to floats and center the readings */
void scaleInputs(float* output) {
  output[0] = float( int16_t( pwm_input[0] ) - PWM_MID );            // roll
  output[1] = float( int16_t( pwm_input[1] ) - PWM_MID );            // pitch
  output[2] = float( int16_t( pwm_input[2] ) - PWM_MIN )*GAIN_THRT;  // throttle
  output[3] = float( int16_t( pwm_input[3] ) - PWM_MID )*GAIN_YAW;   // yaw
}

float positive(float input) {
  return input < 0 ? 0 : input;  
}

//----- Servos

void setupServos() {
    #ifdef USE_TAIL_SERVOS
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    servo[0].attach(2);
    servo[1].attach(3);
    #endif
    
    /* wing servos */
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    servo[2].attach(4);
    servo[3].attach(5);
}

//---- Waveform

/* time variable between 0 and 1 */
float floorTime() {
  float t = micros() * 1e-6 * FREQ;
  return t - floor(t); 
}

/* trucanted triangle wave for times between 0 and 0.5 */
float halfWave(float x, float dm) {
  float m = 4 + dm;
  float im = 1.0/m;
  float x1 = 0.25 - im;
  float x2 = 0.25 + im;
  return x < x1 ? 1.0           :
         x < x2 ? -m*(x - 0.25) : -1.0;  
}

/* complete wave for times between 0 and 1 */
float fullWave(float x, float dm) {
  return x < 0.5 ? halfWave(x, dm) : -halfWave(x - 0.5, dm);
}

/* increase and decrease the slope of time */
float freqMod(float t, float ds) {
  constexpr float CONST = 1.0/float(PWM_MAX - PWM_MIN);  
  ds = ds * CONST;
  float m = 1.0/(1 + ds);        
  float im = 0.5/m;
  return t < im ? m*t : (0.5*t - 0.5)/(1 - im) + 1;
}

/* truncated triangle wave with variable ramp-up and ramp-down */
float fwave(float amp, float ds=0) {
  constexpr float CONST = GAIN_WAVE/(PWM_MAX - PWM_MIN); 
  float dm = positive(amp) * CONST;
  float t  = floorTime();
  float tm = freqMod(t, ds);
  return fullWave(tm, dm);
}

//----- Control mixes

/* v-tail mixing with adjustable pitch and roll gain */
void vmix(float* output, float* input, const float gain_x=1, const float gain_y=1) {
  float input_0 = input[0]*gain_x;
  float input_1 = input[1]*gain_y;
  float mix1 = input_1 - input_0;
  float mix2 = input_1 + input_0;
  // return output
  output[0] = mix1;
  output[1] = mix2;
} 

//----- Cutoff

/* low voltage cutoff based on voltage divider */
bool voltageCutoff() {
  constexpr float CONST = (5.0/1024.0)*(RESISTOR_1 + RESISTOR_2)/float(RESISTOR_1);
  static float mean = VOLT_MAX; 
  float volt = analogRead(A7) * CONST;
  mean += (volt - mean)*1e-4;              // average filter n=1e4
  return mean < VOLT_CUT ? HIGH : LOW;     // cutoff voltage
}

//----- Pitching moment correction

/* elevator trim that increases with throttle. Counters pitch-down moment caused by wing flapping */
float pitchTrim(float x) {
  constexpr float M = PITCH_TRIM_Y / float(PITCH_TRIM_X);
  return x < PITCH_TRIM_X ? M*x : PITCH_TRIM_Y;     
}

//----- Main loop

void setup() {
  setupISR();
  setupServos();
  pinMode(A7, INPUT);
}

void loop() {
  float input[4]; scaleInputs(input);

  #ifdef USE_TAIL_SERVOS
  float mix1[2]; vmix(mix1, input, GAIN_ROLL, GAIN_PITCH);
  
  float ptrim = pitchTrim( input[2] );   
  mix1[0] += ptrim;
  mix1[1] += ptrim;

  servo[0].writeMicroseconds( PWM_MID + mix1[0] + TRIM_TLEFT  );    // left tail
  servo[1].writeMicroseconds( PWM_MID - mix1[1] + TRIM_TRIGHT );    // right tail
  #endif
  
  /* wing servos */
  float mix2[2]; vmix(mix2, input, GAIN_ROLL_DH, GAIN_PITCH_DH);   // differential dihedral
  float mix3[2]; vmix(mix3, input,-GAIN_ROLL_FM, GAIN_PITCH_FM);   // frequency modulation
  
  bool cutoff = voltageCutoff();         // dissable flapping at low voltage 
  float amp1 = cutoff ? 0 : positive( input[2] + input[3] );
  float amp2 = cutoff ? 0 : positive( input[2] - input[3] );
  
  float wave1 = fwave( input[2], mix3[0] ); 
  float wave2 = fwave( input[2], mix3[1] ); 
    
  wave1 = wave1*amp1 + mix2[0];          // differential amplitude
  wave2 = wave2*amp2 + mix2[1];
  
  servo[2].writeMicroseconds( PWM_MID + wave1   + TRIM_LEFT   );    // left wing
  servo[3].writeMicroseconds( PWM_MID - wave2   + TRIM_RIGHT  );    // right wing
}
