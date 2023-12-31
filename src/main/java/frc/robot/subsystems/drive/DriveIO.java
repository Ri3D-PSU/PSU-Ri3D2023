package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double gyroYawRad = 0.0;
        public double gyroYawRadPerSec = 0.0;
        public double leftLeaderCurrent = 0.0;
        public double leftFollowerCurrent = 0.0;
        public double rightLeaderCurrent = 0.0;
        public double rightFollowerCurrent = 0.0;
        public boolean isGyroReady = true;


    }

    /**
     * Updates the set of loggable inputs.
     */
    public default void updateInputs(DriveIOInputs inputs) {
    }

    /**
     * Run open loop at the specified voltage.
     */
    public default void setVoltage(double leftVolts, double rightVolts) {
    }
}
