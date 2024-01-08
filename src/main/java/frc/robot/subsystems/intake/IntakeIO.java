package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeInputs {
        public double primaryIntakeVoltage = 0.0;
        public double primaryIntakeCurrent = 0.0;
        public double primaryIntakeVelocity = 0.0;
        public double primaryIntakePosition = 0.0;
        public double primaryIntakeTemperature = 0.0;


        public double secondaryIntakeCurrent = 0.0;
        public double secondaryIntakeVoltage = 0.0;
        public double secondaryIntakeTemperature = 0.0;
        public double secondaryIntakeVelocity = 0.0;
        public double secondaryIntakePosition = 0.0;

        public boolean isBeamBroken = false;
    }

    /**
     * Updates the set of loggable inputs.
     */
    public default void updateInputs(IntakeInputs inputs) {
    }

    public default void setPrimaryIntakeVelocity(double velocity) {
    }

    public default void setSecondaryIntakeVelocity(double velocity) {
    }
}
