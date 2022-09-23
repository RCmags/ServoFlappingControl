/* ORNITHOPTER SERVO CONTROLLER */

/* Sketch for an ATTiny85 that operates the wing servos of a radio controlled ornithopter. 
 * Author: RCmags https://github.com/RCmags
 * 
 * NOTE: Code is intended to be used with ATTinyCore.
*/

//=============== Connections ================
// See included schematic
  // Inputs:
// Pin 2 -> Receiver CH1
// Pin 3 -> Receiver CH2
// Pin 4 -> Receiver CH3
  // Outputs:
// Pin 0  -> Left  wing servo
// Pin 1  -> Right wing servo 

//=================== Code ===================
#include <Servo_ATTinyCore.h>
#include "parameters.cpp"

//---- global variables 
Servo servo[2]; 
volatile uint16_t pwm_input[3] = {0};

//----- Input signals

/* port change interrupt to read PWM inputs from receiver */
ISR( PCINT0_vect ) {
  static uint32_t initial_time[3] = {0}; 
  static uint8_t port_last = PINB;  
  
  // port changes
  uint32_t current_time = micros(); 
  uint8_t port_rise = ~port_last & PINB;
  uint8_t port_fall = port_last & ~PINB;
  
  // find changing pins
  for( uint8_t index = 0; index < 3; index += 1) {
    uint8_t mask = B00000100 << index;  // Start at PCINT2
    if( port_rise & mask ) {                
        initial_time[index] = current_time;
    } else if ( port_fall & mask ) {       
        pwm_input[index] = current_time - initial_time[index];
    }
  }
  port_last = PINB;    
}

void setupISR() {
  // enable pin change interrupts
  GIMSK |= (1 << PCIE);     
  // set pins as PCINT
  PCMSK |= (1 << PCINT2);   
  PCMSK |= (1 << PCINT3);
  PCMSK |= (1 << PCINT4);
  // set as input
  pinMode(PB2, INPUT_PULLUP);   
  pinMode(PB3, INPUT_PULLUP); 
  pinMode(PB4, INPUT_PULLUP); 
}

//----- Input filter

/* remove noise from PWM inputs then scale and center */
void filterInputs(float* output) {
  static int filter[3] = {0};  
  
  // apply deadband filter
  for( uint8_t index = 0; index < 3; index += 1 ) {
    int change = int( pwm_input[index] ) - filter[index]; 
    filter[index] = change >  INPUT_CHANGE ? int(pwm_input[index]) - INPUT_CHANGE :
                    change < -INPUT_CHANGE ? int(pwm_input[index]) + INPUT_CHANGE : filter[index];
  }
  // scale and center inputs
  output[0] = float( filter[0] - PWM_MID );            // roll
  output[1] = float( filter[1] - PWM_MID );            // pitch
  output[2] = float( filter[2] - PWM_MIN )*GAIN_THRT;  // throttle
}

float positive(float input) {
  return input < 0 ? 0 : input;  
}

//----- Servos

void setupServos() {    
    pinMode(PB0, OUTPUT);
    pinMode(PB1, OUTPUT);
    servo[0].attach(PB0);
    servo[1].attach(PB1);
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

//----- Main loop

void setup() {
  setupISR();
  setupServos();
}

void loop() {
  float input[3]; filterInputs(input);
  float mix1[2]; vmix(mix1, input, GAIN_ROLL_DH, GAIN_PITCH_DH);   // differential dihedral
  float mix2[2]; vmix(mix2, input,-GAIN_ROLL_FM, GAIN_PITCH_FM);   // frequency modulation
  
  float amp1 = positive( input[2] + input[3] );
  float amp2 = positive( input[2] - input[3] );
  
  float wave1 = fwave( input[2], mix2[0] ); 
  float wave2 = fwave( input[2], mix2[1] ); 
    
  wave1 = wave1*amp1 + mix1[0];          // differential amplitude
  wave2 = wave2*amp2 + mix1[1];
  
  servo[0].writeMicroseconds( PWM_MID + wave1 + TRIM_LEFT  );    // left wing
  servo[1].writeMicroseconds( PWM_MID - wave2 + TRIM_RIGHT );    // right wing
}
