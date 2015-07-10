# From levels defined on page 64 of NIST report "AUTONOMY LEVELS FOR UNMANNED SYSTEMS (ALFUS) FRAMEWORK" Verson: 1.0
# Source: http://www.nist.gov/customcf/get_pdf.cfm?pub_id=823618


uint8 LOA_INACTIVE=0        # When the vehicle is powered on but no actuation is enabled.
uint8 LOA_PREARM=1          # When small motors are enabled, such as fixed wing flight surface actuators and gimbal motors.
uint8 LOA_REMOTE_CONTROL=2  # Enables all the motors and requires that enough systems are available for direct manual control of the vehicle.
uint8 LOA_TELEOPERATION=3   # Enables all the motors and requires that enough systems are available for teleoperation.
uint8 LOA_HUMAN_DIRECTED=4  # Enables all the motors and requires that enough systems are available for human directed operations.
uint8 LOA HUMAN_AIDED=5     # Enables all the motors and requires that enough systems are available for human aided operations.
uint8 LOA_AUTONOMOUS=6      # Enables all the motors and requires that enough systems are available for autonomous operation. 

uint8 level_of_autonomy
