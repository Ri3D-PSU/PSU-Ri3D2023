package frc.robot.subsystems.claw;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClawIOSparkMax implements ClawIO {
    CANSparkMax shooterLeft;
    CANSparkMax shooterRight;
    CANSparkMax intake;

    DigitalInput beamBreak;

    public ClawIOSparkMax() {
        shooterLeft = new CANSparkMax(20, CANSparkMax.MotorType.kBrushless);
        shooterRight = new CANSparkMax(21, CANSparkMax.MotorType.kBrushless);
        intake = new CANSparkMax(22, CANSparkMax.MotorType.kBrushless);
        beamBreak = new DigitalInput(0);

        shooterLeft.getEncoder().setPositionConversionFactor(1.0 / 42.0); //TODO
        shooterRight.getEncoder().setPositionConversionFactor(1.0 / 42.0); //TODO
        intake.getEncoder().setPositionConversionFactor(1.0 / 42.0); //TODO

        shooterLeft.getEncoder().setVelocityConversionFactor(1.0 / 42.0); //TODO
        shooterRight.getEncoder().setVelocityConversionFactor(1.0 / 42.0); //TODO
        intake.getEncoder().setVelocityConversionFactor(1.0 / 42.0); //TODO

        shooterLeft.enableVoltageCompensation(10.0);
        shooterRight.enableVoltageCompensation(10.0);
        intake.enableVoltageCompensation(10.0);

        shooterLeft.setSmartCurrentLimit(40);
        shooterRight.setSmartCurrentLimit(40);
        intake.setSmartCurrentLimit(20); //neo 550

        shooterLeft.getPIDController().setP(0.01);
        shooterLeft.getPIDController().setI(0.0);
        shooterLeft.getPIDController().setD(0.0);
        shooterLeft.getPIDController().setFF(0.0);

        shooterRight.getPIDController().setP(0.01);
        shooterRight.getPIDController().setI(0.0);
        shooterRight.getPIDController().setD(0.0);
        shooterRight.getPIDController().setFF(0.0);

        intake.getPIDController().setP(0.01);
        intake.getPIDController().setI(0.0);
        intake.getPIDController().setD(0.0);
        intake.getPIDController().setFF(0.0);
    }

    public void updateInputs(ClawIOInputs inputs) {
        inputs.shooterLeftVoltage = shooterLeft.getBusVoltage();
        inputs.shooterRightVoltage = shooterRight.getBusVoltage();
        inputs.shooterLeftCurrent = shooterLeft.getOutputCurrent();
        inputs.shooterRightCurrent = shooterRight.getOutputCurrent();
        inputs.shooterLeftVelocity = shooterLeft.getEncoder().getVelocity();
        inputs.shooterRightVelocity = shooterRight.getEncoder().getVelocity();
        inputs.shooterLeftPosition = shooterLeft.getEncoder().getPosition();
        inputs.shooterRightPosition = shooterRight.getEncoder().getPosition();
        inputs.shooterLeftTemperature = shooterLeft.getMotorTemperature();
        inputs.shooterRightTemperature = shooterRight.getMotorTemperature();

        inputs.intakeCurrent = intake.getOutputCurrent();
        inputs.intakeVoltage = intake.getBusVoltage();
        inputs.intakeTemperature = intake.getMotorTemperature();
        inputs.intakeVelocity = intake.getEncoder().getVelocity();
        inputs.intakePosition = intake.getEncoder().getPosition();

        inputs.isBeamBroken = !beamBreak.get();
    }

    public void setShooterVelocity(double leftVelocity, double rightVelocity) {
        shooterLeft.getPIDController().setReference(leftVelocity, ControlType.kVelocity);
        shooterRight.getPIDController().setReference(rightVelocity, ControlType.kVelocity);
    }

    public void setIntakeVelocity(double velocity) {
        intake.getPIDController().setReference(velocity, ControlType.kVelocity);
    }
}
