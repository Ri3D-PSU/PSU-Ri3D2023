package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public class ClimberInputs {
        public double climberPosition = 0.0;
        public double climberVelocity = 0.0;
        public double climberCurrent = 0.0;
        public double climberTemperature = 0.0;
        public double climberVoltage = 0.0;

        public double climberFollowerPosition = 0.0;
        public double climberFollowerVelocity = 0.0;
        public double climberFollowerCurrent = 0.0;
        public double climberFollowerTemperature = 0.0;
        public double climberFollowerVoltage = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    public default void updateInputs(ClimberInputs inputs) {

    }

    public default void setPrimaryClimberPower(double voltage) {

    }

    public default void setSecondaryClimberPower(double voltage) {

    }

    public default void setPrimaryClimberPosition(double position, double ffvoltage) {

    }

    public default void setSecondaryClimberPosition(double position, double ffvoltage) {

    }


}
