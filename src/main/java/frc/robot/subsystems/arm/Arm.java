package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO.ArmInputs;

public class Arm extends SubsystemBase {

    ArmIO io;
    ArmInputs inputs = new ArmInputs();

    public Arm(ArmIO io) {
        this.io = io;
    }

    private double targetPosition = 0.0;
    private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0);

    /**
     * @param position The position to set the arm to in degrees
     */
    public void setArmPosition(double position) {
        targetPosition = position;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        io.setArmPosition(targetPosition, feedforward.calculate(inputs.armPositionRad, inputs.armVelocityRadPerSec));
    }

    public double getArmPosition() {
        return inputs.armPositionRad;
    }
}
