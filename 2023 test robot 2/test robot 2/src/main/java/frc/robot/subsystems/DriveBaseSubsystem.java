package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBaseSubsystem extends SubsystemBase{
    private WPI_TalonFX leftmotor1;
    private WPI_TalonFX leftmotor2;
    private WPI_TalonFX rightmotor1;
    private WPI_TalonFX rightmotor2;
    private Solenoid solShifter;
    private Solenoid solBrake;
    private DifferentialDrive difDrive;
    public DriveBaseSubsystem(){
        leftmotor1 = new WPI_TalonFX(KMotorBaseL1);
        leftmotor2 = new WPI_TalonFX(KMotorBaseL2);
        rightmotor1 = new WPI_TalonFX(KMotorBaseR1);
        rightmotor2 = new WPI_TalonFX(KMotorBaseR2);
       // solShifter= new Solenoid(PneumaticsModuleType.REVPH,solCylinderShifterForward);
        //solBrake= new Solenoid(PneumaticsModuleType.REVPH,solCylinderBrakeForward);

        leftmotor2.follow(leftmotor1);
        rightmotor2.follow(rightmotor1);
        difDrive = new DifferentialDrive(leftmotor1, rightmotor1);
    }
   /*  public void extendShifter(){
        solShifter.set(true);
    }

    public void extendBrake(){
        solBrake.set(true);
    }

    public void drive (double x, double y){
        difDrive.arcadeDrive(-x, -y);
    }*/
    public void drive(double x, double y) {
    }
}
