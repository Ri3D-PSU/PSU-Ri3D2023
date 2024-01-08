package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeInputs;

public class Intake extends SubsystemBase {

    IntakeIO io;
    IntakeInputs inputs = new IntakeInputs();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void setPrimaryIntakeVelocity(double leftVelocity) {
        io.setPrimaryIntakeVelocity(leftVelocity);
    }

    public void setSecondaryIntakeVelocity(double velocity) {
        io.setSecondaryIntakeVelocity(velocity);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getPrimaryIntakeVelocity() {
        return inputs.primaryIntakeVelocity;
    }

    public boolean isBeamBroken() {
        return inputs.isBeamBroken;
    }
}
