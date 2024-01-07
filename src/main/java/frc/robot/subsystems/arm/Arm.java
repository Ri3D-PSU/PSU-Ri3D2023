package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO.ArmInputs;

public class Arm extends SubsystemBase {

    ArmIO io;
    ArmInputs inputs = new ArmInputs();

    public Arm(ArmIO io) {
        this.io = io;
    }

    /**
     * @param position The position to set the arm to in degrees
     */
    public void setArmPosition(double position) {
        io.setArmPosition(position);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public double getArmPosition() {
        return inputs.armPosition;
    }
}
