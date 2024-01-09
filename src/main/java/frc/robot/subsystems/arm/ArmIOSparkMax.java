package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import static frc.robot.Constants.RPM_TO_RAD_PER_SEC;

public class ArmIOSparkMax implements ArmIO {

    CANSparkMax arm;
    CANSparkMax armFollower;

    public ArmIOSparkMax() {
        arm = new CANSparkMax(5, MotorType.kBrushless);
        arm.setInverted(true);
        arm.getEncoder().setPositionConversionFactor((1.0 / 36.0) * (16.0 / 62.0));
        arm.getEncoder().setVelocityConversionFactor((1.0 / 36.0) * (16.0 / 62.0) * RPM_TO_RAD_PER_SEC);
        arm.enableVoltageCompensation(11.0);
        arm.setSmartCurrentLimit(40);

        arm.getPIDController().setP(0.005);
        arm.getPIDController().setI(0.0001);
        arm.getPIDController().setD(0.0);
        arm.getPIDController().setFF(0.0);

        arm.setSmartCurrentLimit(40);

        armFollower = new CANSparkMax(3, MotorType.kBrushless);
        armFollower.follow(arm, true);

        arm.setIdleMode(IdleMode.kBrake);
        armFollower.setIdleMode(IdleMode.kBrake);

        arm.getPIDController().setOutputRange(-0.5, 0.5);
    }

    public void updateInputs(ArmInputs inputs) {
        inputs.armPositionRad = arm.getEncoder().getPosition();
        inputs.armVelocityRadPerSec = arm.getEncoder().getVelocity();
        inputs.armCurrent = arm.getOutputCurrent();
        inputs.armTemperature = arm.getMotorTemperature();
        inputs.armVoltage = arm.getBusVoltage();
    }

    public void setArmPosition(double position, double ffvoltage) {
        arm.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0,
                ffvoltage, ArbFFUnits.kVoltage);
    }
}
