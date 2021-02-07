# ServoFlappingControl
This Arduino sketch is the servo controller for an RC ornithopter (Flapping MAV) powered and controlled by two servos. That is, each wing is flapped independently by one or more servos.

The program is designed to receive 3 PWM inputs from an RC receiver (operating in MODE 2) and outputs 2 PWM signals that can be understood by off-the-shelft, hobby grade servos. The aileron (1) and elevator (2) channels are used to bias the dihedral angle of each wing, with the aileron rotating the wings in unison, while the elevator changes the dihedral angle. The throttle channel (3) control the amplitude of a fixed-frequency oscillation. There are 3 wave-forms the user can select: sine wave, triangle wave, and saw wave. 

NOTE: MODE 1 can also be used with this program. It is only a matter of swapping the RX connections to the arduino.

The program was written for an Arduino Nano but it should be compatible with other boards. It also requires the "PinChageInterrupt" library found here:
  https://www.arduino.cc/reference/en/libraries/pinchangeinterrupt/

For more information on the controller, please see this post:
  https://www.rcgroups.com/forums/showpost.php?p=41325203&postcount=69
