package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveBaseSubsystem extends SubsystemBase {
    private CANSparkMax leftmotor1;
    private CANSparkMax leftmotor2;
    private CANSparkMax rightmotor1;
    private CANSparkMax rightmotor2;
    private Solenoid solShifter;
    private Solenoid solBrake;
    private DifferentialDrive difDrive;
    public DriveBaseSubsystem(){
        leftmotor1 = new CANSparkMax(KMotorBaseL1, MotorType.kBrushless);
        leftmotor2 = new CANSparkMax(KMotorBaseL2,MotorType.kBrushless);
        rightmotor1 = new CANSparkMax(KMotorBaseR1,MotorType.kBrushless);
        rightmotor2 = new CANSparkMax(KMotorBaseR2,MotorType.kBrushless);
        solShifter= new Solenoid(PneumaticsModuleType.REVPH,solCylinderShifterForward);
        solBrake= new Solenoid(PneumaticsModuleType.REVPH,solCylinderBrakeForward);

        leftmotor2.follow(leftmotor1);
        rightmotor2.follow(rightmotor1);
        difDrive = new DifferentialDrive(leftmotor1, rightmotor1);
    }
    public void extendShifter(){
        solShifter.set(true);
    }

    public void extendBrake(){
        solBrake.set(true);
    }

    public void drive (double x, double y){
        difDrive.arcadeDrive(-x, -y);
    }
}
