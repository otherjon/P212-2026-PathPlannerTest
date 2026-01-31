#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import logging
log = logging.Logger('P212-robot')

import wpilib
import commands2

import robotcontainer


class Robot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be
        used for any initialization code.
        """
        self.autonomousCommand = None

        # Instantiate our RobotContainer.  This will perform all our button
        # bindings, and put our autonomous chooser on the dashboard.
        self.container = robotcontainer.RobotContainer()

        log.info('robot initialized')

    def autonomousInit(self) -> None:
        """
        This method runs the autonomous command selected by your
        RobotContainer class.
        """
        self.autonomousCommand = self.container.get_autonomous_command()

        # schedule the autonomous command, if any
        if self.autonomousCommand is not None:
            self.autonomousCommand.schedule()
        else:
            log.warning("No autonomous command")

    def teleopInit(self) -> None:
        # Cancel the running autonomous command, if any
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()

        # If any subsystem has a teleopInit() method, run that
        for ss in self.container.all_subsystems():
            if hasattr(ss, "teleopInit"):
                ss.teleopInit()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        # If any subsystem has a teleopPeriodic() method, run that
        for ss in self.container.all_subsystems():
            if hasattr(ss, "teleopPeriodic"):
                ss.teleopPeriodic()

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(Robot)
