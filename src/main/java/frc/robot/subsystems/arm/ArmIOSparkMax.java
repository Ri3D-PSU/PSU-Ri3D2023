package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;

public class ArmIOSparkMax implements ArmIO {

    CANSparkMax arm;

    public ArmIOSparkMax() {
        arm = new CANSparkMax(30, CANSparkMax.MotorType.kBrushless);
        arm.getEncoder().setPositionConversionFactor(1.0 / 42.0); //TODO
        arm.getEncoder().setVelocityConversionFactor(1.0 / 42.0); //TODO
        arm.enableVoltageCompensation(10.0);
        arm.setSmartCurrentLimit(40);

        arm.getPIDController().setP(0.01);
        arm.getPIDController().setI(0.0);
        arm.getPIDController().setD(0.0);
        arm.getPIDController().setFF(0.0);

        arm.setSmartCurrentLimit(40);

    }

    public void updateInputs(ArmInputs inputs) {
        inputs.armPosition = arm.getEncoder().getPosition();
        inputs.armVelocity = arm.getEncoder().getVelocity();
        inputs.armCurrent = arm.getOutputCurrent();
        inputs.armTemperature = arm.getMotorTemperature();
        inputs.armVoltage = arm.getBusVoltage();
    }

    public void setArmPosition(double position) {
        arm.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
    }
}
