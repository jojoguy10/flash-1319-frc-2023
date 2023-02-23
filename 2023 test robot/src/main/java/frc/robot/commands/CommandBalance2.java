package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class CommandBalance2 extends CommandBase {
    private Robot robot;
    public CommandBalance2(Robot robot) {
        super();
        // Allows the command to use parts from the robot
        this.robot = robot;
    }
    
    // Run this code every 20 milliseconds when the command is running
    @Override
    public void execute() {
        // True level of the IMU based off of the initial pitch at start of auto
        double truePitch = robot.imu.getPitch()- robot.startPitch;
        // The angle before the robot starts moving again
        double limit = 4; // -4 to 4 degrees
        // If the pitch is less than the lower limit
        if(truePitch < -limit) {
            // drive forward
            robot.m_robotDrive.arcadeDrive(0.1, 0, false);
        // else if the pitch is greater than limit
        } else if (truePitch > limit) {
            // drive backward
            robot.m_robotDrive.arcadeDrive(-0.1, 0, false);
        } else {
            // The angle is within the limit, stop the robot
            robot.m_robotDrive.arcadeDrive(0, 0, false);
        }
    }

    // Check if the command has balanced the platform
    @Override
    public boolean isFinished() {
        // Check if the pitch of the robot is within plus or minus 1 degree of being level
        //return Math.abs(robot.imu.getPitch() - robot.startPitch) < 1;
        return false;
    }

    // When the command finishes, run this code
    @Override
    public void end(boolean interrupted) {
        // Stop the robots base
        robot.m_robotDrive.arcadeDrive(0, 0);
    }
}
