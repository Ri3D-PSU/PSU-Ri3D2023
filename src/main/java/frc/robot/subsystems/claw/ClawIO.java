package frc.robot.subsystems.claw;

public interface ClawIO {
    public static class ClawIOInputs {
        public double shooterLeftVoltage = 0.0;
        public double shooterRightVoltage = 0.0;
        public double shooterLeftCurrent = 0.0;
        public double shooterRightCurrent = 0.0;
        public double shooterLeftVelocity = 0.0;
        public double shooterRightVelocity = 0.0;
        public double shooterLeftPosition = 0.0;
        public double shooterRightPosition = 0.0;
        public double shooterLeftTemperature = 0.0;
        public double shooterRightTemperature = 0.0;


        public double intakeCurrent = 0.0;
        public double intakeVoltage = 0.0;
        public double intakeTemperature = 0.0;
        public double intakeVelocity = 0.0;
        public double intakePosition = 0.0;

        public boolean isBeamBroken = false;
    }

    /**
     * Updates the set of loggable inputs.
     */
    public default void updateInputs(ClawIOInputs inputs) {
    }

    public default void setShooterVelocity(double leftVelocity, double rightVelocity) {
    }

    public default void setIntakeVelocity(double velocity) {
    }
}
