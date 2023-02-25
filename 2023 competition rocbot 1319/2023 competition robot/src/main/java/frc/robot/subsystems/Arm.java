package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private CANSparkMax shoulderM1 = new CANSparkMax(5, MotorType.kBrushless);
    // Telescoping Motor
    private CANSparkMax telescopingM1 = new CANSparkMax(9, MotorType.kBrushless);
    SparkMaxLimitSwitch bottomLimitShoulder;
    SparkMaxLimitSwitch topLimitShoulder;
    SparkMaxLimitSwitch bottomLimitTelescoping;
    SparkMaxLimitSwitch topLimitTelescoping;
    double teleMidPos = 23;
    double teleHighPos = 120;
    public Arm() {
        bottomLimitShoulder = shoulderM1.getReverseLimitSwitch(Type.kNormallyOpen);
        bottomLimitShoulder.enableLimitSwitch(true);
        topLimitShoulder = shoulderM1.getForwardLimitSwitch(Type.kNormallyOpen);
        topLimitShoulder.enableLimitSwitch(true);

        bottomLimitTelescoping = telescopingM1.getReverseLimitSwitch(Type.kNormallyOpen);
        bottomLimitTelescoping.enableLimitSwitch(true);
        topLimitTelescoping = telescopingM1.getForwardLimitSwitch(Type.kNormallyOpen);
        topLimitTelescoping.enableLimitSwitch(true);

        double shoulderP = 0.01;
        double shoulderI = 0.0001;
        double shoulderD = 0.01;
        SparkMaxPIDController pidShoulder = shoulderM1.getPIDController();
        pidShoulder.setP(shoulderP);
        pidShoulder.setI(shoulderI);
        pidShoulder.setIMaxAccum(0.5, 0);
        pidShoulder.setD(shoulderD);
        pidShoulder.setOutputRange(-1.0, 1.0);

        double teleP = 0.01;
        double teleI = 0.0000;
        double teleD = 0.00;
        SparkMaxPIDController pidTele = telescopingM1.getPIDController();
        pidTele.setP(teleP);
        pidTele.setI(teleI);
        pidTele.setIMaxAccum(0.5, 0);
        pidTele.setD(teleD);
        pidTele.setOutputRange(-1.0, 1.0);

        setShoulderAngle(0);
    }

    @Override
    public void periodic() {
        // Reset the encoder count if the bottom limit switch is pressed
        if(bottomLimitShoulder.isPressed()) {
            shoulderM1.getEncoder().setPosition(0);
        }

        if(bottomLimitTelescoping.isPressed()) {
            telescopingM1.getEncoder().setPosition(0);
        }
        
        SmartDashboard.putNumber("Tele motor 1 encoder.", telescopingM1.getEncoder().getPosition());
        SmartDashboard.putNumber("  Shoulder motor 1 encoder.", shoulderM1.getEncoder().getPosition());
        SmartDashboard.putBoolean("shoulder limit switch",
                shoulderM1.getReverseLimitSwitch(Type.kNormallyOpen).isPressed());
    }

    public void setShoulderAngle(double angle) {
        shoulderM1.getPIDController().setReference(angle, ControlType.kPosition);
    }

    public void setTelescopePos(double pos) {
        telescopingM1.getPIDController().setReference(pos, ControlType.kPosition);
    }

    public void teleopPeriodic(XboxController Driver, XboxController Operator) {
        // Start Button
        if (Operator.getStartButton()) {
            setTelescopePos(80);
            // Back Button
        } else if (Operator.getBackButton()) {
            setTelescopePos(0);
        } else {
            //telescopingM1.set(0);
        }
        
        if (Operator.getPOV() == 0) {
            setShoulderAngle(121);
        } else if (Operator.getPOV() == 180) {
            setShoulderAngle(0);
        } else {
            //shoulderM1.set(0);
        }
    }
}
