/*      CONSTANT     |    VALUE   |  UNIT  |   DESCRIPTION */
/*=========================================================*/
// 1.  Flapping waveform
#define FREQ              2.8     // hz     // Wingbeat frequency [hz]. Use lowest frequency that achieves flight
#define GAIN_WAVE         12.0              /* Triangle wave to square wave gain (truncation). 
                                               A larger gain makes the waveform become more square-like with more throttle */
// 2. Control inputs [negate to reverse direction]
    // I. Base inputs:
#define GAIN_PITCH        1.0               // Tail elevator
#define GAIN_ROLL         1.0               // Tail aileron
#define GAIN_THRT         0.8               // Flapping amplitude. Increases with more throttle
#define GAIN_YAW          0.4               // Differential amplitude. Controlled by rudder
    // II. Differential dihedral:
#define GAIN_PITCH_DH     0.0               // Pitch: symmetric dihedral
#define GAIN_ROLL_DH      0.0               // Roll: asymmetric dihedral
    // III. Frequency modulation:
#define GAIN_PITCH_FM     0.0               // Pitch: wings fall at the same speed
#define GAIN_ROLL_FM      0.0               // Roll: One wing falls faster than the other

// 3. Servo trims 
#define TRIM_LEFT         0       // us     // left  wing 
#define TRIM_RIGHT        -60     // us     // right wing
#define TRIM_TLEFT        0       // us     // left  tail
#define TRIM_TRIGHT       50      // us     // right tail

// 4. Pitching moment correction 
#define PITCH_TRIM_X      200     // us     // Throttle PWM when trim reaches maximum value 
#define PITCH_TRIM_Y      -350    // us     // Maximum pitch trim. Negate to reverse deflection

// 5. Low voltage cutoff [voltage divider]
#define RESISTOR_1        67000   // ohm    // Resistor from ground -> middle 
#define RESISTOR_2        118000  // ohm    // Resistor from middle -> voltage
#define VOLT_CUT          6.0     // volt   // cutoff voltage
#define VOLT_MAX          8.4     // volt   // fully charged voltage 

// 6. Output PWM signal [all servos]
#define PWM_MID           1500    // us     // Center/middle servo position
#define PWM_MIN           1100    // us     // Minimum servo position
#define PWM_MAX           1900    // us     // Maximum servo position

// 7. Other
#define USE_TAIL_SERVOS                     // Comment line to dissable tail servo output (only control wings)
