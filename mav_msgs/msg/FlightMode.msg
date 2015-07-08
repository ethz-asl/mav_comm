uint8 MODE_INACTIVE=0 # When the vehicle is powered on but no actuation is enabled.
uint8 MODE_PREARM=1   # When small motors are enabled, such as fixed wing flight surface actuators and gimbal motors.
uint8 MODE_MANUAL=2   # Enables all the motors and requires that enough systems are available for direct manual control of the vehicle.
uint8 MODE_RATE=3     # Enables all the motors and requires that enough systems are available for rate based control from a pilot.
uint8 MODE_ATTITUDE=4 # Enables all the motors and requires that enough systems are available for attitude control from a pilot.
uint8 MODE AUTO=5     # Enables all the motors and requires that enough systems are available for autonomous operations.
uint8 MODE_RTL=6      # Enables all the motors and asks the autopilot to effect Return To Launch procedure.

uint8 mode
