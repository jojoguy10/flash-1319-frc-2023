package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private DoubleSolenoid m_intake = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 2, 4);
    private Solenoid m_gripper = new Solenoid(10, PneumaticsModuleType.REVPH, 3);
    private DigitalInput gripperLimitSwitch = new DigitalInput(9);
    boolean gripperDebounce;
    public double gripperClosedTime = 0;
    // Intake Motors
    //private final CANSparkMax intakeM1 = new CANSparkMax(7, MotorType.kBrushed);
    //private final CANSparkMax intakeM2 = new CANSparkMax(8, MotorType.kBrushed);
    boolean intakeRunning;
    public IntakeSubsystem() {
        //intakeM2.setInverted(true);
        //m_intake.set(Value.kReverse);
        gripperDebounce = false;
        gripperClosedTime = 0;
    }

    public void toggleIntake() {
        //m_intake.toggle();
    }

    public void toggleGripper() {
        m_gripper.toggle();
    }

    public void closeGripper() {
        m_gripper.set(false);
    }

    public void lowerIntake() {
        //m_intake.set(Value.kForward);
    }

    public void runIntakeForwards() {
        //intakeM1.set(1);
        //intakeM2.set(1);
    }

    public void runIntakeBackwards() {
        //intakeM1.set(-1);
        //intakeM2.set(-1);
    }

    public void stopIntake() {
        //intakeM1.set(0);
        //intakeM2.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Solenoid", m_intake.get() == Value.kForward); // Green out, Red in
        SmartDashboard.putBoolean("Gripper", m_gripper.get()); // Red close, Green open
        if(!m_gripper.get()) {
            // Gripper is closed
            gripperClosedTime += 0.02;
        } else {
            gripperClosedTime = 0;
        }
        /*if(!gripperLimitSwitch.get() == true) {
            if(!gripperDebounce) {
                m_gripper.set(false);
                gripperDebounce = true;
            }
        } else {
            gripperDebounce = false;
        }*/
    }
}
