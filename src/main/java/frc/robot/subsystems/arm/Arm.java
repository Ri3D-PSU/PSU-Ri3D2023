package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

    ArmIO io;
    ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

    public Arm(ArmIO io) {
        this.io = io;
    }

    private double targetPosition = 0.0;
    private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.577
            , 0.0);

    /**
     * @param position The position to set the arm to in degrees
     */
    public void setArmPositionDegrees(double position) {
        targetPosition = Math.toRadians(position);
    }

    public double getTargetArmPosition() {
        return targetPosition;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);
        Logger.getInstance().recordOutput("Arm Angle", Math.toDegrees(inputs.armPositionRad));
        Logger.getInstance().recordOutput("Target Arm Angle", Math.toDegrees(targetPosition));

        double armffvoltage = feedforward.calculate(inputs.armPositionRad, inputs.armVelocityRadPerSec);
        Logger.getInstance().recordOutput("ArmFFVoltage", armffvoltage);

        io.setArmPosition(targetPosition, armffvoltage);
    }

    public double getArmPosition() {
        return inputs.armPositionRad;
    }

    public void adjustAnglePosition(double degrees) {
        io.adjustArmAngle(Math.toRadians(degrees));
    }

    public void resetAngle(double radians) {

    }
}
