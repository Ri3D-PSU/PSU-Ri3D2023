package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmInputs {
        public double armPositionRad = 0.0;
        public double armVelocityRadPerSec = 0.0;
        public double armCurrent = 0.0;
        public double armTemperature = 0.0;
        public double armVoltage = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    public default void updateInputs(ArmInputs inputs) {

    }

    public default void setArmPosition(double position, double ffvoltage) {

    }
}
