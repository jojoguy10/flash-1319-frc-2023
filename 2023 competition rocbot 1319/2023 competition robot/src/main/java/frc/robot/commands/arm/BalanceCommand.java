package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class BalanceCommand extends CommandBase {
    private Drivetrain robot;
    double timeBalanced = 0;
    public BalanceCommand(Drivetrain robot) {
        super();
        // Allows the command to use parts from the robot
        this.robot = robot;
    }

    @Override
    public void initialize() {
      timeBalanced = 0;
    }
    
    // Run this code every 20 milliseconds when the command is running
    @Override
    public void execute() {
        // True level of the IMU based off of the initial pitch at start of auto
        double truePitch = robot.imu.getRoll()- robot.startPitch;
        // The angle before the robot starts moving again
        double limit = 8; // -4 to 4 degrees
        double factor = 1;//Math.abs(robot.imu.getPitch()-robot.startPitch)/0.8;
        // If the pitch is less than the lower limit
        if(truePitch < -limit) {
            timeBalanced = 0;
            // drive forward
            robot.arcadeDrive(-0.13*factor, 0, false);
        // else if the pitch is greater than limit
        } else if (truePitch > limit) {
            // drive backward
            timeBalanced += 0.02;
            robot.arcadeDrive(0.13*factor, 0, false);
        } else {
            timeBalanced = 0;
            // The angle is within the limit, stop the robot
            robot.arcadeDrive(0, 0, false);
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
        robot.arcadeDrive(0, 0, false);
    }
}
