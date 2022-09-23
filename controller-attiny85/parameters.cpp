// 1.  Flapping waveform
#define FREQ              2.8          // Wingbeat frequency [hz]. Use lowest frequency that achieves flight
#define GAIN_WAVE         12.0         /* Triangle wave to square wave gain (truncation). 
                                          A larger gain makes the waveform become more square-like with more throttle */
                                       
// 2. Control inputs [negate to reverse direction]
    // I. Base inputs:
#define GAIN_THRT         0.8          // Flapping amplitude. Increases with more throttle
#define GAIN_YAW          0.4          // Differential amplitude. Controlled by rudder
    // II. Differential dihedral:
#define GAIN_PITCH_DH     0.5          // Pitch: symmetric dihedral
#define GAIN_ROLL_DH      0.5          // Roll: asymmetric dihedral
    // III. Frequency modulation:
#define GAIN_PITCH_FM     0.0          // Pitch: wings fall at the same speed
#define GAIN_ROLL_FM      0.0          // Roll: One wing falls faster than the other

// 3. Servo trims [ms]
#define TRIM_LEFT         0            // left  wing 
#define TRIM_RIGHT        -60          // right wing

// 6. Output PWM signal
#define PWM_MID           1500         // Center/middle servo position
#define PWM_MIN           1100         // Minimum servo position
#define PWM_MAX           1900         // Maximum servo position

// 7. Input deadband filter
#define INPUT_CHANGE      12           // Change in PWM signal needed to update receiver inputs
