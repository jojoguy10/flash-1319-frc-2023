package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//sol means a Pneumatic cylinder.
public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax armMotor1;
    private CANSparkMax armMotor2;
    private Solenoid solArm;
    public ArmSubsystem(){
        armMotor1 = new CANSparkMax(KMotorBaseA1,MotorType.kBrushless);
        armMotor2 = new CANSparkMax(KMotorBaseA2,MotorType.kBrushless);

        solArm = new Solenoid(PneumaticsModuleType.REVPH,solCylinderArmForward);
    }
    public void extendArm(){
        solArm.set(true);
    }
    public void toggleArm(){
        solArm.toggle();
    }
    public double getEncoder(){
    return armMotor1.getEncoder().getPosition() + armMotor2.getEncoder().getPosition()/2;
    }
    
    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("in Encoder position", getEncoder());
    }
}
