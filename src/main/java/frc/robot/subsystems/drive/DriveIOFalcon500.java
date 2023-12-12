package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;

public class DriveIOFalcon500 implements DriveIO {
    private static final double GEAR_RATIO = 6.0;
    private static final double TICKS_PER_REV = 2048;

    private final TalonFX leftLeader;
    private final TalonFX rightLeader;
    private final TalonFX leftFollower;
    private final TalonFX rightFollower;

    private final AHRS gyro;

    public DriveIOFalcon500() {
        leftLeader = new TalonFX(3);
        rightLeader = new TalonFX(1);
        leftFollower = new TalonFX(4);
        rightFollower = new TalonFX(2);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.statorCurrLimit.enable = true;
        config.statorCurrLimit.currentLimit = 40;
        leftLeader.configAllSettings(config);
        rightLeader.configAllSettings(config);

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        leftLeader.setInverted(false);
        rightLeader.setInverted(true);
        leftFollower.setInverted(InvertType.FollowMaster);
        rightFollower.setInverted(InvertType.FollowMaster);

        leftFollower.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);
        leftLeader.setNeutralMode(NeutralMode.Brake);
        rightLeader.setNeutralMode(NeutralMode.Brake);

        gyro = new AHRS();
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftPositionRad = Units.rotationsToRadians(
                leftLeader.getSelectedSensorPosition() / TICKS_PER_REV / GEAR_RATIO);
        inputs.rightPositionRad = Units.rotationsToRadians(
                rightLeader.getSelectedSensorPosition() / TICKS_PER_REV / GEAR_RATIO);
        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
                leftLeader.getSelectedSensorVelocity() * 10 / TICKS_PER_REV / GEAR_RATIO);
        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
                rightLeader.getSelectedSensorVelocity() * 10 / TICKS_PER_REV / GEAR_RATIO);
        inputs.gyroYawRad = Math.toRadians(gyro.getYaw());
        inputs.leftLeaderCurrent = leftLeader.getSupplyCurrent();
        inputs.leftFollowerCurrent = leftFollower.getSupplyCurrent();
        inputs.rightLeaderCurrent = rightLeader.getStatorCurrent();
        inputs.rightFollowerCurrent = rightFollower.getStatorCurrent();
        inputs.isGyroReady = gyro.isConnected() && !gyro.isCalibrating();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftLeader.set(ControlMode.PercentOutput, leftVolts / 12.0);
        rightLeader.set(ControlMode.PercentOutput, rightVolts / 12.0);
    }
}
