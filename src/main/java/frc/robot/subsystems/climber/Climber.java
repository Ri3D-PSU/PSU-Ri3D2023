package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Climber extends SubsystemBase {

    ClimberIO io;
    ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    LoggedDashboardNumber climberStartPosition = new LoggedDashboardNumber("Climb Start Position", 32);

    public Climber(ClimberIO io) {
        this.io = io;
    }

    enum ClimberState {
        IDLE,
        CLIMBING,
        TargetPosition,

    }

    private ClimberState state = ClimberState.IDLE;
    private double targetPower = 0.0;
    private double targetPosition = 0.0;

    public void setClimberPower(double voltage) {
        state = ClimberState.CLIMBING;
        targetPower = voltage;
    }

    public void stopClimber() {
        state = ClimberState.IDLE;
        targetPower = 0.0;
        targetPosition = getClimberPosition();
    }

    public void extendClimber() {
        state = ClimberState.TargetPosition;
        targetPower = 0.0;
        targetPosition = 32;
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Climber", inputs);

        Logger.getInstance().recordOutput("ClimberState", state.toString());
        Logger.getInstance().recordOutput("ClimberTargetPower", targetPower);
        Logger.getInstance().recordOutput("ClimberTargetPosition", targetPosition);

        switch (state) {
            case IDLE -> {
                io.setPrimaryClimberPower(0.0);
                io.setSecondaryClimberPower(0.0);
            }
            case CLIMBING -> {
                io.setPrimaryClimberPower(targetPower);
                io.setSecondaryClimberPosition(inputs.climberPosition, targetPower); // PID the follower to the primary
            }
            case TargetPosition -> {
                io.setPrimaryClimberPosition(targetPosition, 0.0);
                io.setSecondaryClimberPosition(targetPosition, 0.0);
            }
        }
    }

    public double getClimberVelocity() {
        return (inputs.climberVelocity + inputs.climberFollowerVelocity) / 2d;
    }

    public double getClimberPosition() {
        return (inputs.climberPosition + inputs.climberFollowerPosition) / 2d;
    }
}
