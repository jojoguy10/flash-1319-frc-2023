package frc.robot.subsystems;

import java.lang.annotation.RetentionPolicy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.CloseGripper;
import frc.robot.commands.arm.ExtendArmToPos;
import frc.robot.commands.arm.FullRetractArm;
import frc.robot.commands.arm.LowerIntake;
import frc.robot.commands.arm.ShoulderDriveToPosition;

public class Arm extends SubsystemBase {
    private CANSparkMax shoulderM1 = new CANSparkMax(5, MotorType.kBrushless);
    // Telescoping Motor
    private CANSparkMax telescopingM1 = new CANSparkMax(9, MotorType.kBrushless);
    SparkMaxLimitSwitch bottomLimitShoulder;
    SparkMaxLimitSwitch topLimitShoulder;
    SparkMaxLimitSwitch bottomLimitTelescoping;
    SparkMaxLimitSwitch topLimitTelescoping;
    double armTarget = 0;
    double teleTarget = 0;

    boolean extended = false;

    // Angle of arm position before the arm must be fully retracted in order to go down further
    public static final double extendLimitThreshold = 80;
    public static final double teleRetractedThresh = 4;
    public static final double teleMidPos = 23;
    public static final double teleHighPos = 120;
    public static final double armHighPos = 125;
    public static final double armMidPos = 110;

    public static enum TelePreset {
        RETRACTED(0),
        MID(teleMidPos),
        HIGH(teleHighPos);
        public final double telePos;

        private TelePreset(double x) {
            this.telePos = x;
        }
    }
    TelePreset curTele;
    public Arm() {
        bottomLimitShoulder = shoulderM1.getReverseLimitSwitch(Type.kNormallyOpen);
        bottomLimitShoulder.enableLimitSwitch(true);
        topLimitShoulder = shoulderM1.getForwardLimitSwitch(Type.kNormallyOpen);
        topLimitShoulder.enableLimitSwitch(true);

        bottomLimitTelescoping = telescopingM1.getReverseLimitSwitch(Type.kNormallyOpen);
        bottomLimitTelescoping.enableLimitSwitch(true);
        topLimitTelescoping = telescopingM1.getForwardLimitSwitch(Type.kNormallyOpen);
        topLimitTelescoping.enableLimitSwitch(true);
        telescopingM1.setSoftLimit(SoftLimitDirection.kForward, 120);

        // Setup the PID for the shoulder motor
        double shoulderP = 0.01;
        double shoulderI = 0.000025;
        double shoulderD = 0.01;
        SparkMaxPIDController pidShoulder = shoulderM1.getPIDController();
        pidShoulder.setP(shoulderP);
        pidShoulder.setI(shoulderI);
        pidShoulder.setIMaxAccum(0.5, 0);
        pidShoulder.setD(shoulderD);
        pidShoulder.setOutputRange(-1.0, 1.0);

        // Setup the PID for the telescoping motor
        double teleP = 0.01;
        double teleI = 0.00001;
        double teleD = 0.00;
        SparkMaxPIDController pidTele = telescopingM1.getPIDController();
        pidTele.setP(teleP);
        pidTele.setI(teleI);
        pidTele.setIMaxAccum(0.2, 0);
        pidTele.setD(teleD);
        pidTele.setOutputRange(-1.0, 1.0);

        // Set the default target angle to zero degrees
        setShoulderAngle(0);
        setTelescopePos(0);
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
        
        if(getArmAngle() < extendLimitThreshold) {
            curTele = TelePreset.RETRACTED;
        } else if(getArmAngle() < armHighPos - 7.5) {
            curTele = TelePreset.MID;
        } else {
            curTele = TelePreset.HIGH;
        }

        SmartDashboard.putNumber("Shoulder Current", shoulderM1.getOutputCurrent());
        SmartDashboard.putNumber("Telescoping Current", telescopingM1.getOutputCurrent());

        SmartDashboard.putNumber("Tele motor 1 encoder.", telescopingM1.getEncoder().getPosition());
        SmartDashboard.putNumber("Shoulder motor 1 encoder.", shoulderM1.getEncoder().getPosition());
        SmartDashboard.putBoolean("shoulder limit switch",
                shoulderM1.getReverseLimitSwitch(Type.kNormallyOpen).isPressed());
        SmartDashboard.putNumber("Shoulder Target", armTarget);
        SmartDashboard.putNumber("Tele Target", teleTarget);
    }

    public TelePreset getTelePreset() {
        return curTele;
    }

    public void setShoulderAngle(double angle) {
        shoulderM1.getPIDController().setReference(angle, ControlType.kPosition);
        armTarget = angle;
    }

    public void setTelescopePos(double pos) {
        telescopingM1.getPIDController().setReference(pos, ControlType.kPosition);
        teleTarget = pos;
    }

    public boolean getRetractedLimitSwitch() {
        return bottomLimitTelescoping.isPressed();
    }

    // Encoder position of the arm
    public double getArmAngle() {
        return shoulderM1.getEncoder().getPosition();
    }

    public double getArmTarget() {
        return armTarget;
    }

    public double getTeleTarget() {
        return teleTarget;
    }

    // Encoder position of the telescoping arm
    public double getTelePos() {
        return telescopingM1.getEncoder().getPosition();
    }
    // is the telescoping arm retracted or nots
    public boolean isExtended() {
        return getTelePos() > teleRetractedThresh;
    }

    public void teleopPeriodic(XboxController Driver, XboxController Operator) {
        // Start Button
        if (Operator.getYButton() && getArmAngle() > extendLimitThreshold) {
            setTelescopePos(curTele.telePos);
            // Back Button
        } else if (Operator.getAButton()) {
            setTelescopePos(0);
        } else {
            //telescopingM1.set(0);
        }
        if(getArmAngle()<extendLimitThreshold){
            curTele = TelePreset.RETRACTED;
        }else if(getArmAngle()< armHighPos -7.5){
            curTele = TelePreset.MID; 
        }else{
            curTele = TelePreset.HIGH;
        }
    }

    public Command createDriveArmCommand(IntakeSubsystem intake, double pos, TelePreset extendPos) {
        return 
            (new LowerIntake(intake))
            //.andThen(new CloseGripper(intake))
            .andThen(new WaitCommand(2.0))
             // Retract the arm unless it is above the point where it does not need to be retracted
            //((new CloseGripper(intake)).andThen(new LowerIntake(intake)).andThen(new FullRetractArm(this))).unless(() -> this.extendLimitThreshold < pos)
            // Then, drive the shoulder to the specified position
            .andThen(new ShoulderDriveToPosition(this, pos));
            // Finally, extend the arm to the new position unless the arm is retracted
            //.andThen(new ExtendArmToPos(this, extendPos).unless(() -> !isExtended()));
    }

    public Command fullLowerCommands(IntakeSubsystem intake) {
        return (new CloseGripper(intake))
            .andThen(new LowerIntake(intake))
            .andThen(new FullRetractArm(this))
            .andThen(new WaitCommand(2.0))
            .andThen(new ShoulderDriveToPosition(this, 0));
    }

    public Command createExtendArm() {
        return (new ExtendArmToPos(this, curTele)).unless(() -> getArmAngle() < this.extendLimitThreshold && curTele != TelePreset.RETRACTED);
    }
}
