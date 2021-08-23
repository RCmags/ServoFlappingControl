# ServoFlappingControl
This Arduino sketch is the servo controller for an RC ornithopter (Flapping MAV) powered and controlled by two servos. Each servo is connected to a wing and flapped independently.

The program is designed to receive 3 PWM inputs from an RC receiver operating in [MODE 2](https://www.rc-airplane-world.com/rc-transmitter-modes.html) and outputs 2 PWM signals for off-the-shelft hobby servos. The aileron (1) and elevator (2) channels are used to bias the dihedral angle of each wing, with the aileron rotating the wings in unison, while the elevator changes the dihedral angle. The throttle channel (3) control the amplitude of a fixed-frequency oscillation. 

There are 3 wave-forms the user can select: sine wave, triangle wave, and saw wave. The controller also is equipped with a low-voltage cutoff routine. This will dissable the throttle channel to prevent over-discharging a battery. The routine uses an analog-pin to read the input voltage via a voltage-divider. 

The program was written for an Arduino Nano but it should be compatible with other boards. 
It requires the [PinChageInterrupt](https://www.arduino.cc/reference/en/libraries/pinchangeinterrupt/) library.
For more information on the controller, please see this [RCgroups post](https://www.rcgroups.com/forums/showpost.php?p=41325203&postcount=69).

See the attached schematic for an example of the required circuit:

<img src = "ReceiverServoFlap_VoltCutoff.png" width = "80%">

This video shows what the controller and how it works:  
[<img src="https://img.youtube.com/vi/T6NfZD_iuEs/maxresdefault.jpg" width="50%">](https://youtu.be/T6NfZD_iuEs)

Lastly, here is an example of the kind of aircraft this code was writen for:

<img src = "/example_pics/servo_body_res.jpg" width = "30%"></img>
<img src = "/example_pics/bottom_view_res.JPG" width = "30%"></img>
<img src = "/example_pics/side_view_res.JPG" width = "30%"></img>

<img src = "/example_pics/flap_motion.gif" width = "30%"></img>
<img src = "/example_pics/roll_motion.gif" width = "30%"></img>
<img src = "/example_pics/pitch_motion.gif" width = "30%"></img>
