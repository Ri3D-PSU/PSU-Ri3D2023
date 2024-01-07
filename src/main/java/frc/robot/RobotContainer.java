// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.dacubeking.AutoBuilder.robot.GuiAuto;
import com.dacubeking.AutoBuilder.robot.annotations.AutoBuilderAccessible;
import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.claw.Claw;
import frc.robot.subsystems.claw.ClawIO;
import frc.robot.subsystems.claw.ClawIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOFalcon500;
import frc.robot.subsystems.drive.DriveIOSim;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private Drive drive;
    private Claw claw;
    private Arm arm;
    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<String> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final LoggedDashboardChooser<String> sideChooser = new LoggedDashboardChooser<>("Side Chooser");

    private final LoggedDashboardNumber flywheelSpeed = new LoggedDashboardNumber("Flywheel Speed", 1500.0);
    private final LoggedDashboardNumber shootAngle = new LoggedDashboardNumber("Shoot Angle", 40);
    private final LoggedDashboardNumber pickupAngle = new LoggedDashboardNumber("Pickup Angle", -20);
    private final LoggedDashboardNumber trapAngle = new LoggedDashboardNumber("Trap Angle", 100);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            // Real robot, instantiate hardware IO implementations
            case REAL -> {
                drive = new Drive(new DriveIOFalcon500());
                claw = new Claw(new ClawIOSparkMax());
                arm = new Arm(new ArmIOSparkMax());
            }

            // drive = new Drive(new DriveIOFalcon500());
            // Sim robot, instantiate physics sim IO implementations
            case SIM -> {
                drive = new Drive(new DriveIOSim());
                claw = new Claw(new ClawIOSparkMax());
                arm = new Arm(new ArmIOSparkMax());
            }

            // Replayed robot, disable IO implementations
            default -> {
                drive = new Drive(new DriveIO() {
                });
                claw = new Claw(new ClawIO() {
                });
                arm = new Arm(new ArmIO() {
                });
            }
        }

        // Configure the button bindings
        configureButtonBindings();

        claw.setDefaultCommand(new RunCommand(
                () -> {
                    claw.setShooterVelocity(0, 0);
                    claw.setIntakeVelocity(0);
                }, claw
        ));

        arm.setDefaultCommand(new RunCommand(
                () -> arm.setArmPosition(arm.getArmPosition()),
                arm
        ));

        // Initialize autonomous container
        AutonomousContainer.getInstance().setDebugPrints(true);
        AutonomousContainer.getInstance().initialize(
                false,
                info -> new RamseteCommand(
                        info.trajectory(),
                        drive::getPose,
                        new RamseteController(),
                        drive.getFeedforward(),
                        drive.getKinematics(),
                        drive::getWheelSpeeds,
                        new PIDController(0, 0, 0),
                        new PIDController(0, 0, 0),
                        drive::driveVoltage,
                        drive
                ),
                drive::resetOdometry,
                false,
                this
        );

        // Set up auto routines
        sideChooser.addDefaultOption("Blue", "blue");
        sideChooser.addOption("Red", "red");

        AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));
    }


    @AutoBuilderAccessible
    Command intakeCommand = new RunCommand(
            () -> {
                claw.setIntakeVelocity(0.5);
                arm.setArmPosition(pickupAngle.get());
            }, claw, arm
    );

    @AutoBuilderAccessible
    Command outtakeCommand = new RunCommand(
            () -> {
                claw.setIntakeVelocity(-0.5);
            }, claw
    );

    @AutoBuilderAccessible
    Command shootCommand = new RunCommand(
            () -> {
                claw.setShooterVelocity(flywheelSpeed.get(), flywheelSpeed.get());
                arm.setArmPosition(shootAngle.get());
            }, claw, arm)
            .until(() -> Math.abs(arm.getArmPosition() - shootAngle.get()) < 5
                    && Math.abs(claw.getShooterVelocity() - flywheelSpeed.get()) < 100)
            .andThen(() -> {
                claw.setIntakeVelocity(1000);
                claw.setShooterVelocity(flywheelSpeed.get(), flywheelSpeed.get());
                arm.setArmPosition(shootAngle.get());
            }, claw, arm);

    @AutoBuilderAccessible
    Command scoreTrapCommand = new RunCommand(() -> {
        arm.setArmPosition(trapAngle.get());
        claw.setIntakeVelocity(0);
    }, arm, claw)
            .until(() -> Math.abs(arm.getArmPosition() - trapAngle.get()) < 5)
            .andThen(() -> claw.setIntakeVelocity(100), arm, claw);


    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        drive.setDefaultCommand(
                new RunCommand(() -> drive.driveArcade(-controller.getLeftY(), -controller.getRightX()), drive));
        controller.leftBumper()
                .whileTrue(new RunCommand(
                        () -> drive.driveArcade(-controller.getLeftY() / 2.0, -controller.getRightX() / 2.0),
                        drive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public @Nullable GuiAuto getAutonomousCommand() {
        return AutonomousContainer.getInstance().getAuto(autoChooser.get(), sideChooser.get(), true);
    }


}
