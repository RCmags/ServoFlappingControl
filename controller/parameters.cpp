// 1.  Flapping waveform
#define FREQ              2.8          // Wingbeat frequency [hz]. Use lowest frequency that achieves flight
#define GAIN_WAVE         12.0         /* Triangle wave to square wave gain (truncation). 
                                          A larger gain makes the waveform become more square-like with more throttle */
                                       
// 2. Control inputs [negate to reverse direction]
    // I. Base inputs:
#define GAIN_PITCH        1.0          // Tail elevator
#define GAIN_ROLL         1.0          // Tail aileron
#define GAIN_THRT         0.8          // Flapping amplitude. Increases with more throttle
#define GAIN_YAW          0.4          // Differential amplitude. Controlled by rudder
    // II. Differential dihedral:
#define GAIN_PITCH_DH     0.0          // Pitch: symmetric dihedral
#define GAIN_ROLL_DH      0.0          // Roll: asymmetric dihedral
    // III. Frequency modulation:
#define GAIN_PITCH_FM     0.0          // Pitch: wings fall at the same speed
#define GAIN_ROLL_FM      0.0          // Roll: One wing falls faster than the other

// 3. Servo trims [ms]
#define TRIM_LEFT         0            // left  wing 
#define TRIM_RIGHT        -60          // right wing
#define TRIM_TLEFT        0            // left  tail
#define TRIM_TRIGHT       50           // right tail

// 4. Pitching moment correction [ms]
#define PITCH_TRIM_X      200          // Throttle PWM when trim reaches maximum value 
#define PITCH_TRIM_Y      -350         // Maximum pitch trim. Negate to reverse deflection

// 5. Low voltage cutoff [voltage divider]
#define RESISTOR_1        67000        // Resistor from ground -> middle [ohms]
#define RESISTOR_2        118000       // Resistor from middle -> voltage [ohms]
#define VOLT_CUT          6.0          // cutoff voltage
#define VOLT_MAX          8.4          // fully charged voltage 

// 6. Output PWM signal [all servos]
#define PWM_MID           1500         // Center/middle servo position
#define PWM_MIN           1100         // Minimum servo position
#define PWM_MAX           1900         // Maximum servo position

// 7. Other
#define USE_TAIL_SERVOS                // Comment line to dissable tail servo output (only control wings)
