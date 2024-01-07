package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);

    private final DriveIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
    private boolean isFirstRun = true;

    /**
     * Creates a new Drive.
     */
    public Drive(DriveIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive", inputs);

        // Update odometry and log the new pose
        if (isFirstRun && inputs.isGyroReady) {
            System.out.println("Gyro Angle When Resetting = " + (-inputs.gyroYawRad));
            odometry.resetPosition(new Rotation2d(-inputs.gyroYawRad), getLeftPositionMeters(),
                    getRightPositionMeters(), new Pose2d());
            isFirstRun = false;
        }
        odometry.update(getHeading(), getLeftPositionMeters(), getRightPositionMeters());
        Logger.getInstance().recordOutput("Odometry", getPose());
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-inputs.gyroYawRad);
    }

    /**
     * Run open loop at the specified percentage.
     */
    public void drivePercent(double leftPercent, double rightPercent) {
        io.setVoltage(leftPercent * 12.0, rightPercent * 12.0);
    }

    /**
     * Run open loop at the specified percentage.
     */
    public void driveVoltage(double leftVoltage, double rightVoltage) {
        io.setVoltage(leftVoltage, rightVoltage);
    }

    /**
     * Run open loop based on stick positions.
     */
    public void driveArcade(double xSpeed, double zRotation) {
        var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, true);
        io.setVoltage(speeds.left * 12.0, speeds.right * 12.0);
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        io.setVoltage(0.0, 0.0);
    }

    /**
     * Returns the current odometry pose in meters.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the position of the left wheels in meters.
     */
    public double getLeftPositionMeters() {
        return inputs.leftPositionRad * WHEEL_RADIUS_METERS;
    }

    /**
     * Returns the position of the right wheels in meters.
     */
    public double getRightPositionMeters() {
        return inputs.rightPositionRad * WHEEL_RADIUS_METERS;
    }

    /**
     * Returns the velocity of the left wheels in meters/second.
     */
    public double getLeftVelocityMeters() {
        return inputs.leftVelocityRadPerSec * WHEEL_RADIUS_METERS;
    }

    /**
     * Returns the velocity of the right wheels in meters/second.
     */
    public double getRightVelocityMeters() {
        return inputs.rightVelocityRadPerSec * WHEEL_RADIUS_METERS;
    }

    public Pose2d getPoseMeters() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getHeading(), getLeftPositionMeters(), getRightPositionMeters(), pose);
    }

    public SimpleMotorFeedforward getFeedforward() {
        return new SimpleMotorFeedforward(0.0, 0.7, 0.2); //Temp values
    }

    public DifferentialDriveKinematics getKinematics() {
        return new DifferentialDriveKinematics(0.6); //Temp values
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMeters(), getRightVelocityMeters());
    }
}
