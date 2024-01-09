package frc.robot.subsystems.arm;

import static frc.robot.Constants.RPM_TO_RAD_PER_SEC;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

public class ArmIOSparkMax implements ArmIO {

    CANSparkMax arm;
    CANSparkMax armFollower;

    public ArmIOSparkMax() {
        arm = new CANSparkMax(5, MotorType.kBrushless);
        arm.setInverted(true);
        arm.getEncoder().setPositionConversionFactor((1.0 / 36) * (64.0 / 16.0) / 2.2);
        arm.getEncoder().setVelocityConversionFactor((1.0 / 36) * (64.0 / 16.0) / 2.2 * RPM_TO_RAD_PER_SEC);
        arm.enableVoltageCompensation(11.0);
        arm.setSmartCurrentLimit(40);

        arm.getPIDController().setP(0.4);
        arm.getPIDController().setI(0.0001);
        arm.getPIDController().setD(0.0);
        arm.getPIDController().setFF(0.0);

        arm.setSmartCurrentLimit(30);

        armFollower = new CANSparkMax(3, MotorType.kBrushless);
        armFollower.follow(arm, true);

        // TODO
        arm.setIdleMode(IdleMode.kBrake);
        armFollower.setIdleMode(IdleMode.kBrake);

        arm.getPIDController().setOutputRange(-0.4, 0.4);
        arm.setClosedLoopRampRate(0.25);

        arm.getEncoder().setPosition(Math.toRadians(-11));
    }

    public void updateInputs(ArmInputs inputs) {
        inputs.armPositionRad = arm.getEncoder().getPosition();
        inputs.armVelocityRadPerSec = arm.getEncoder().getVelocity();
        inputs.armCurrent = arm.getOutputCurrent();
        inputs.armTemperature = arm.getMotorTemperature();
        inputs.armVoltage = arm.getAppliedOutput() * arm.getBusVoltage();
    }

    public void setArmPosition(double position, double ffvoltage) {
        arm.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0,
                ffvoltage, ArbFFUnits.kVoltage);
    }

    @Override
    public void adjustArmAngle(double angleRadians) {
        arm.getEncoder().setPosition(arm.getEncoder().getPosition() + angleRadians);
    }

    @Override
    public void armAngle(double angleRadians) {
        arm.getEncoder().setPosition(angleRadians);
    }
}
