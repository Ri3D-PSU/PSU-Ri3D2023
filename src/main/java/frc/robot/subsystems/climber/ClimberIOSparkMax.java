package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClimberIOSparkMax implements ClimberIO {

    CANSparkMax climberMotor;
    CANSparkMax climberFollowerMotor;

    public ClimberIOSparkMax() {
        climberMotor = new CANSparkMax(50, MotorType.kBrushless);
        climberFollowerMotor = new CANSparkMax(51, MotorType.kBrushless);

        climberMotor.getEncoder().setPositionConversionFactor(1.0 / 42.0); //TODO
        climberFollowerMotor.getEncoder().setPositionConversionFactor(1.0 / 42.0); //TODO

        climberMotor.getEncoder().setVelocityConversionFactor(1.0 / 42.0); //TODO
        climberFollowerMotor.getEncoder().setVelocityConversionFactor(1.0 / 42.0); //TODO

        climberMotor.getPIDController().setP(0.01);
        climberMotor.getPIDController().setI(0.0001);
        climberMotor.getPIDController().setD(0.0);

        climberFollowerMotor.getPIDController().setP(0.01);
        climberFollowerMotor.getPIDController().setI(0.0001);
        climberFollowerMotor.getPIDController().setD(0.0);
    }

    public void updateInputs(ClimberInputs inputs) {
        inputs.climberPosition = climberMotor.getEncoder().getPosition();
        inputs.climberVelocity = climberMotor.getEncoder().getVelocity();
        inputs.climberCurrent = climberMotor.getOutputCurrent();
        inputs.climberTemperature = climberMotor.getMotorTemperature();
        inputs.climberVoltage = climberMotor.getBusVoltage();

        inputs.climberFollowerPosition = climberFollowerMotor.getEncoder().getPosition();
        inputs.climberFollowerVelocity = climberFollowerMotor.getEncoder().getVelocity();
        inputs.climberFollowerCurrent = climberFollowerMotor.getOutputCurrent();
        inputs.climberFollowerTemperature = climberFollowerMotor.getMotorTemperature();
        inputs.climberFollowerVoltage = climberFollowerMotor.getBusVoltage();
    }

    public void setPrimaryClimberPower(double voltage) {
        climberMotor.set(voltage);
    }

    public void setSecondaryClimberPower(double voltage) {
        climberFollowerMotor.set(voltage);
    }

    public void setPrimaryClimberPosition(double position, double ffvoltage) {
        climberMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0, ffvoltage);
    }

    public void setSecondaryClimberPosition(double position, double ffvoltage) {
        climberFollowerMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0, ffvoltage);
    }
}
