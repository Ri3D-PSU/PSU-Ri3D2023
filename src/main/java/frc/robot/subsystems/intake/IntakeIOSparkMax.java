package frc.robot.subsystems.intake;

import static frc.robot.Constants.RPM_TO_RAD_PER_SEC;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeIOSparkMax implements IntakeIO {
    CANSparkMax primaryIntake;
    CANSparkMax secondaryIntake;

    public IntakeIOSparkMax() {
        primaryIntake = new CANSparkMax(7, MotorType.kBrushless);
        secondaryIntake = new CANSparkMax(8, MotorType.kBrushless);

        primaryIntake.getEncoder().setPositionConversionFactor(1.0 / 5.0);
        secondaryIntake.getEncoder().setPositionConversionFactor(1.0 / 20.0);

        primaryIntake.getEncoder().setVelocityConversionFactor((1.0 / 5.0) * RPM_TO_RAD_PER_SEC);
        secondaryIntake.getEncoder().setVelocityConversionFactor((1.0 / 20.0) * RPM_TO_RAD_PER_SEC);

        primaryIntake.enableVoltageCompensation(10.0);
        secondaryIntake.enableVoltageCompensation(10.0);

        primaryIntake.setSmartCurrentLimit(40);
        secondaryIntake.setSmartCurrentLimit(20); //neo 550

        primaryIntake.getPIDController().setP(0.00);
        primaryIntake.getPIDController().setI(0.00001);
        primaryIntake.getPIDController().setD(0.0);
        primaryIntake.getPIDController().setFF(0.0);

        secondaryIntake.getPIDController().setP(0.00);
        secondaryIntake.getPIDController().setI(0.00001);
        secondaryIntake.getPIDController().setD(0.0);
        secondaryIntake.getPIDController().setFF(0.0);

        primaryIntake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        secondaryIntake.setIdleMode(CANSparkMax.IdleMode.kBrake);

    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
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
