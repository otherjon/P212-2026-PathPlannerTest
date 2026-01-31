import logging
log = logging.Logger('P212-robot')

import commands2
import constants

## TODO: Change this for your robot!
##       (Change the import line so that it imports your subsystem by its
##        correct name.)
##
from subsystems.example_ss import ExampleSubsystem


## TODO: Change this for your robot!
##       (Give your command class a descriptive class name.)
##
class TemplateCommand(commands2.Command):
    ## TODO: Change this for your robot!
    ##       (Write reasonable documentation for your command.)
    ##
    """
    This class is a template for how commands for your robot should be
    structured.
    """
    ## TODO: Change this for your robot!
    ##       (Change the name and class of the constructor's subsystem
    ##        parameter.)
    ##
    def __init__(self, example_ss: ExampleSubsystem) -> None:
        """
        Constructor for the command object.  Assigns some instance variables.
        """
        super().__init__()
        ## TODO: Change this for your robot!
        ##       (Assign the correct named parameter to a sensibly named
        ##        instance variable.)
        ##
        self.example_ss = example_ss

        # addRequirements() declares that this command needs exclusive use of
        # this subsystem.  If another command that needs this subsystem gets
        # scheduled to run, this command won't be able to run anymore, so it
        # will get cancelled.  For example, commands such as RaiseElevator and
        # LowerElevator both use the Elevator subsystem and can't run at the
        # same time, so they must each call addRequirements(self.elevator_ss)
        #
        self.addRequirements(self.example_ss)

    def initialize(self):
        """
        Perform any setup to initialize the command, and/or perform any
        command that can be completed all in one shot.
        This method runs when the scheduler schedules the command.
        """
        ## TODO: Change this for your robot!
        ##       (Can this command do everything in one shot?  If not, does
        ##        this command need to do anything to set up?  If so, put that
        ##        code here.)
        ##
        self.example_ss.activate_actuator()

    def execute(self):
        """
        Performs the main part of any command that needs to happen on an
        ongoing basis, such as continuously reading a joystick.
        This method runs 50 times a second while the command is active.
        """
        ## TODO: Change this for your robot!
        ##       (What does this command need to do continuously?  Put that
        ##        code here.  If you don't need to do anything continuously,
        ##        you can delete the entire execute() method.)
        ##

    def isFinished(self):
        """
        Returns a boolean indicating whether the command has completed.
        """
        ## TODO: Change this for your robot!
        ##       (What test determines whether this command has completed?  If
        ##        you did everything in initialize(), then then command has
        ##        already completed (and will always have completed), so you
        ##        can just return True.)
        ##

        # stop the motor if the sensor value is over 5.0
        return self.example_ss.sensor_value() > 5.0

    def end(self, interrupted: bool):
        """
        Perform any cleanup or final steps needed after the command finishes.
        If you need to do something different depending on whether the command
        completed normally or was interrupted (by another command), the
        :interrupted: parameter will be True if the command was interrupted.
        """
        ## TODO: Change this for your robot!
        ##       (Does your command need to do anything once at the end?  If
        ##        so, put that code here.  If you don't need to do anything,
        ##        you can delete the entire end() method.)
        ##

        self.example_ss.stop()

