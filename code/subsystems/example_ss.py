import logging
log = logging.Logger('P212-robot')

from wpilib import DigitalInput
import commands2
import rev
from constants import ELEC

## TODO: Change this for your robot!
##       (Import the libraries you need.)
import rev


## TODO: Change this for your robot!
##       (Change the name of the subsystem.  Use InitialCapitals.)
##
class ExampleSubsystem(commands2.Subsystem):
    """
    This class represents an example subsystem for your robot.  Edit it to
    model your actual subsystems.
    """
    def __init__(self) -> None:
        """Creates a new ExampleSubsystem"""
        super().__init__()
        # Create sensors here, and assign them to instance variables.
        # (Define the DIO port or CAN bus ID that your sensor uses in
        #  constants.py)

        ## TODO: Change this for your robot!
        ##       (Use your sensors and constants, and change the variable name.)
        self.my_sensor = DigitalInput(ELEC.my_sensor_DIO_port)

        # Create actuators here, and assign them to instance variables.
        # (Define the DIO port or CAN bus ID that your actuator uses in
        #  constants.py)

        ## TODO: Change this for your robot!
        ##       (Use your actuators and constants; change the variable name.)
        self.my_motor = rev.SparkMax(
            ELEC.my_motor_CAN_ID, rev.SparkMax.MotorType.kBrushless)

    ## TODO: Change these methods for your robot!
    ##       (Write methods that define what your subsystem does.)
    ##
    def sensor_value(self):
        """
        Example method that reads a subsystem sensor.
        """
        return self.my_sensor.get()

    def activate_actuator(self, output_value):
        """
        Example method that activates a subsystem actuator
        """
        self.my_motor.set(output_value)

    def stop(self):
        """
        Example method that stops everything on the robot
        """
        self.my_motor.set(0.0)

