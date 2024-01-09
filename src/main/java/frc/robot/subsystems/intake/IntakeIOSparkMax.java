package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO {
    CANSparkMax primaryIntake;
    CANSparkMax secondaryIntake;

    DigitalInput beamBreak;

    public IntakeIOSparkMax() {
        primaryIntake = new CANSparkMax(20, MotorType.kBrushless);
        secondaryIntake = new CANSparkMax(22, MotorType.kBrushless);
        beamBreak = new DigitalInput(0); // TODO: Does this exist?

        primaryIntake.getEncoder().setPositionConversionFactor(1.0 / 42.0); //TODO
        secondaryIntake.getEncoder().setPositionConversionFactor(1.0 / 42.0); //TODO

        primaryIntake.getEncoder().setVelocityConversionFactor(1.0 / 42.0); //TODO
        secondaryIntake.getEncoder().setVelocityConversionFactor(1.0 / 42.0); //TODO

        primaryIntake.enableVoltageCompensation(10.0);
        secondaryIntake.enableVoltageCompensation(10.0);

        primaryIntake.setSmartCurrentLimit(40);
        secondaryIntake.setSmartCurrentLimit(20); //neo 550

        primaryIntake.getPIDController().setP(0.01);
        primaryIntake.getPIDController().setI(0.0);
        primaryIntake.getPIDController().setD(0.0);
        primaryIntake.getPIDController().setFF(0.0);

        secondaryIntake.getPIDController().setP(0.01);
        secondaryIntake.getPIDController().setI(0.0);
        secondaryIntake.getPIDController().setD(0.0);
        secondaryIntake.getPIDController().setFF(0.0);

    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.isBeamBroken = beamBreak.get();

        inputs.primaryIntakeCurrent = primaryIntake.getOutputCurrent();
        inputs.primaryIntakeVoltage = primaryIntake.getBusVoltage();
        inputs.primaryIntakeVelocity = primaryIntake.getEncoder().getVelocity();
        inputs.primaryIntakePosition = primaryIntake.getEncoder().getPosition();
        inputs.primaryIntakeTemperature = primaryIntake.getMotorTemperature();

        inputs.secondaryIntakeCurrent = secondaryIntake.getOutputCurrent();
        inputs.secondaryIntakeVoltage = secondaryIntake.getBusVoltage();
        inputs.secondaryIntakeVelocity = secondaryIntake.getEncoder().getVelocity();
        inputs.secondaryIntakePosition = secondaryIntake.getEncoder().getPosition();
        inputs.secondaryIntakeTemperature = secondaryIntake.getMotorTemperature();
    }

    public void setPrimaryIntakeVelocity(double velocity) {
        primaryIntake.getPIDController().setReference(velocity, ControlType.kVelocity);
    }

    public void setSecondaryIntakeVelocity(double velocity) {
        secondaryIntake.getPIDController().setReference(velocity, ControlType.kVelocity);
    }
}
