package frc.robot.subsystems.arm;

public interface ArmIO {
    public static class ArmIOInputs {
        public double armPosition = 0.0;
        public double armVelocity = 0.0;
        public double armCurrent = 0.0;
        public double armTemperature = 0.0;
        public double armVoltage = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    public default void updateInputs(ArmIOInputs inputs) {

    }

    public default void setArmPosition(double position) {

    }
}
