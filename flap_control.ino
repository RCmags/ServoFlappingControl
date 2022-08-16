#include <Servo.h>

//---- Input parameters

#define FREQ          3.0          // wingbeat frequency, hz
#define GAIN_WAVE     10.0         // triangle wave trucation gain

#define GAIN_PITCH    1.0
#define GAIN_ROLL     1.0          
#define GAIN_THRT     0.8          // variable amplitude
#define GAIN_YAW      0.4          // differential amplitude

#define GAIN_PITCH_DH 0.0          // differential dihedral, pitch
#define GAIN_ROLL_DH  0.0          // differential dihedral, roll

#define GAIN_PITCH_FM 0.0          // frequency modulation, pitch
#define GAIN_ROLL_FM  0.0          // frequency modulation, roll

#define TRIM_LEFT     0            // left wing 
#define TRIM_RIGHT    -60          // right wing
#define TRIM_TLEFT    0            // left tail
#define TRIM_TRIGHT   50           // right tail
      
#define RESISTOR_1    67000        // ground -> middle
#define RESISTOR_2    118000       // middle -> voltage
#define VOLT_CUT      6.0          // cutoff voltage
#define VOLT_MAX      8.4          // fully charged voltage

#define PWM_MID       1500
#define PWM_MIN       1100
#define PWM_MAX       1900

//---- global variables 

Servo    servo[4]; 
volatile uint16_t pwm_input[4] = {0};

//----- Input signals

// PORTB = {8 .. 13} -> using pins {9 .. 12} = B00011110

ISR( PCINT0_vect ) {
  // store state
  static uint32_t initial_time[4] = { micros() };  
  static uint8_t  port_last = B00000000;
  
  // scan pins
  uint8_t port_falling = ~PINB & port_last;
  uint32_t current_time = micros(); 
  
  for( uint8_t index = 0; index < 4; index += 1) {
    uint8_t mask = B00000010 << index;
    if( PINB & mask ) {                 // rising pin
      initial_time[index] = current_time;
    } else if( port_falling & mask ) {  // falling pin
      pwm_input[index] = current_time - initial_time[index];
    }    
  } 
  port_last = PINB; // store last state
}

void setupISR() {
  PCICR = B00000001;    // enable PORTB interrupts
  PCMSK0 = B00011110;   // enable ISR for pins 9-12
  for ( uint8_t pin  = 9; pin <= 12; pin++ ) { 
    pinMode(pin, INPUT_PULLUP);
  }  
}

//----- Input filter

void scaleInputs(float* output) {
  output[0] = float( int16_t( pwm_input[0] ) - PWM_MID );            // roll
  output[1] = float( int16_t( pwm_input[1] ) - PWM_MID );            // pitch
  output[2] = float( int16_t( pwm_input[2] ) - PWM_MIN )*GAIN_THRT;  // throttle
  output[3] = float( int16_t( pwm_input[3] ) - PWM_MID )*GAIN_YAW;   // yaw
}

float positive(float input) {
  return input < 0 ? 0 : input;           // lift is linear with amplitude in forward flight
}

//----- Servos

void setupServos() {
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    servo[0].attach(2);      // left  tail
    servo[1].attach(3);      // right tail
    servo[2].attach(4);      // left  wing
    servo[3].attach(5);      // right wing
}

//---- Waveform

float floorTime() {
  float t = micros() * 1e-6 * FREQ;
  return t - floor(t); 
}

float halfWave(float x, float dm) {
  float m = 4 + dm;
  float im = 1.0/m;
  float x1 = 0.25 - im;
  float x2 = 0.25 + im;
  return x < x1 ? 1.0           :
         x < x2 ? -m*(x - 0.25) :
                 -1.0           ;  
}

float fullWave(float x, float dm) {
  return x < 0.5 ? halfWave(x, dm) : -halfWave(x - 0.5, dm);
}

float freqMod(float t, float xi) {
  constexpr float CONST = 1.0/float(PWM_MAX - PWM_MIN);  
  xi = xi * CONST;
  float m = 1.0/(1 + xi);        
  float im = 0.5/m;
  return t < im ? m*t : (0.5*t - 0.5)/(1 - im) + 1 ;
}

float fwave(float amp, float ds=0) {
  constexpr float CONST = GAIN_WAVE/(PWM_MAX - PWM_MIN); 
  float dm = positive(amp) * CONST;
  float t  = floorTime();
  float tm = freqMod(t, ds);
  return fullWave(tm, dm);
}

//----- Control mixes

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

bool voltageCutoff() {
  constexpr float CONST = (5.0/1024.0)*(RESISTOR_1 + RESISTOR_2)/RESISTOR_1;
  static float mean = VOLT_MAX; 
  float volt = analogRead(A7) * CONST;
  mean += (volt - mean)*1e-4;              // average filter n=1e4
  return mean < VOLT_CUT ? HIGH : LOW;     // cutoff voltage
}

//----- Main code

void setup() {
  setupISR();
  setupServos();
  pinMode(A7, INPUT);
}

void loop() {
  float input[4]; scaleInputs(input);
  float mix1[2]; vmix(mix1, input, GAIN_ROLL   , GAIN_PITCH);      // v-tail
  float mix2[2]; vmix(mix2, input, GAIN_ROLL_DH, GAIN_PITCH_DH);   // differential dihedral
  float mix3[2]; vmix(mix3, input,-GAIN_ROLL_FM, GAIN_PITCH_FM);   // frequency modulation
  
  bool cutoff = voltageCutoff();
  float amp1 = cutoff ? 0 : positive( input[2] + input[3] );
  float amp2 = cutoff ? 0 : positive( input[2] - input[3] );
  
  float wave1 = fwave( input[2], mix3[0] ); 
  float wave2 = fwave( input[2], mix3[1] ); 
    
  wave1 = wave1*amp1 + mix2[0];          // differential amplitude
  wave2 = wave2*amp2 + mix2[1];
  
  servo[0].writeMicroseconds( PWM_MID + mix1[0] + TRIM_TLEFT  );    // left  tail
  servo[1].writeMicroseconds( PWM_MID - mix1[1] + TRIM_TRIGHT );    // right tail
  servo[2].writeMicroseconds( PWM_MID + wave1   + TRIM_LEFT   );    // left  wing
  servo[3].writeMicroseconds( PWM_MID - wave2   + TRIM_RIGHT  );    // right wing
}

// Note: need to combine assymetries for robust control
