package frc.robot.subsystems.drive;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public class DriveIOSim implements DriveIO {
    private DifferentialDrivetrainSim sim = DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDoubleFalcon500PerSide,
            KitbotGearing.k10p71, KitbotWheelSize.kSixInch,
            new MatBuilder(Nat.N7(), Nat.N1()).fill(0.005, 0.005, 0.0001, 0.05, 0.05, 0.005, 0.005));

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        sim.update(0.02);
        inputs.leftPositionRad = sim.getLeftPositionMeters() / Drive.WHEEL_RADIUS_METERS;
        inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / Drive.WHEEL_RADIUS_METERS;
        inputs.rightPositionRad = sim.getRightPositionMeters() / Drive.WHEEL_RADIUS_METERS;
        inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond() / Drive.WHEEL_RADIUS_METERS;
        inputs.gyroYawRad = sim.getHeading().getRadians() * -1;
        inputs.isGyroReady = true;
        inputs.leftLeaderCurrent = sim.getLeftCurrentDrawAmps();
        inputs.leftFollowerCurrent = sim.getLeftCurrentDrawAmps();
        inputs.rightLeaderCurrent = sim.getRightCurrentDrawAmps();
        inputs.rightFollowerCurrent = sim.getRightCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        sim.setInputs(MathUtil.clamp(leftVolts, -12.0, 12.0), MathUtil.clamp(rightVolts, -12.0, 12.0));
    }
}
