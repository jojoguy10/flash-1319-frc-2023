package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class CommandBalance extends CommandBase {
    private PIDController pid = new PIDController(0.02, 0.005, 0.0);
    private Robot robot;
    public CommandBalance(Robot robot) {
        super();
        // Allows the command to use parts from the robot
        this.robot = robot;
    }
    
    // Run this code every 20 milliseconds when the command is running
    @Override
    public void execute() {
        // Calculate the PID's output for the motors, based off the pitch of the robot,
        // accounting for the starting pitch
        
        double output = pid.calculate(robot.imu.getPitch()- robot.startPitch);
        if(output > 0.25) {
            output = 0.25;
        } else if(output < -0.25) {
            output = -0.25;
        }
        SmartDashboard.putNumber("PID OUTPUT", output);
        // Use the calculated output to drive the robot forward and backwards to reach the target
        robot.m_robotDrive.arcadeDrive(output, 0, false);
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
