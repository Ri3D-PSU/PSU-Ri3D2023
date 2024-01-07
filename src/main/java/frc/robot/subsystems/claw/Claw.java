package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.claw.ClawIO.ClawInputs;

public class Claw extends SubsystemBase {

    ClawIO io;
    ClawInputs inputs = new ClawInputs();

    public Claw(ClawIO io) {
        this.io = io;
    }

    public void setShooterVelocity(double leftVelocity, double rightVelocity) {
        io.setShooterVelocity(leftVelocity, rightVelocity);
    }

    public void setIntakeVelocity(double velocity) {
        io.setIntakeVelocity(velocity);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getShooterVelocity() {
        return (inputs.shooterLeftVelocity + inputs.shooterRightVelocity) / 2;
    }
}
