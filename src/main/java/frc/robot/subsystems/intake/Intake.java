package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    IntakeIO io;
    IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void setPrimaryIntakeVelocity(double velocity) {
        io.setPrimaryIntakeVelocity(velocity);
        Logger.getInstance().recordOutput("PrimaryIntakeTargetVelocity", velocity);
    }

    public void setSecondaryIntakeVelocity(double velocity) {
        io.setSecondaryIntakeVelocity(velocity);
        Logger.getInstance().recordOutput("SecondaryIntakeTargetVelocity", velocity);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);
    }

    public double getPrimaryIntakeVelocity() {
        return inputs.primaryIntakeVelocity;
    }

    public boolean isBeamBroken() {
        return inputs.isBeamBroken;
    }

    public double getSecondaryIntakeCurrent() {
        return inputs.secondaryIntakeCurrent;
    }

    public double getPrimaryIntakeCurrent() {
        return inputs.primaryIntakeCurrent;
    }
}
