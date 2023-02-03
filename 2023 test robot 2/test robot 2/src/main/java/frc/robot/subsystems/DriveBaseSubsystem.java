package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CommandDriveBase;

public class DriveBaseSubsystem extends SubsystemBase{
    private WPI_TalonFX leftmotor1;
    private WPI_TalonFX leftmotor2;
    private WPI_TalonFX rightmotor1;
    private WPI_TalonFX rightmotor2;
    private DifferentialDrive difDrive;

    public DriveBaseSubsystem(){
        leftmotor1 = new WPI_TalonFX(KMotorBaseL1);
        leftmotor2 = new WPI_TalonFX(KMotorBaseL2);
        rightmotor1 = new WPI_TalonFX(KMotorBaseR1);
        rightmotor2 = new WPI_TalonFX(KMotorBaseR2);

        leftmotor2.follow(leftmotor1);
        rightmotor2.follow(rightmotor1);
        difDrive = new DifferentialDrive(leftmotor1, rightmotor1);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    public void drive (double x, double y){
        difDrive.arcadeDrive(-x, -y);
    }
}
