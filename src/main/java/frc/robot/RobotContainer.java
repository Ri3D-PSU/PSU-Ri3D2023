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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOFalcon500;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
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
    private Intake intake;
    private Arm arm;
    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<String> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final LoggedDashboardChooser<String> sideChooser = new LoggedDashboardChooser<>("Side Chooser");

    private final LoggedDashboardNumber shootSpeed = new LoggedDashboardNumber("Shoot Speed", 1500.0);
    private final LoggedDashboardNumber shootAngle = new LoggedDashboardNumber("Shoot Angle", 40);
    private final LoggedDashboardNumber pickupAngle = new LoggedDashboardNumber("Pickup Angle", -20);
    private final LoggedDashboardNumber ampScoringAngle = new LoggedDashboardNumber("Amp Scoring Angle", 40);
    private final LoggedDashboardNumber ampScoringSpeed = new LoggedDashboardNumber("Amp Scoring Speed", 1000.0);
    private final LoggedDashboardNumber intakeSpeed = new LoggedDashboardNumber("Intake Speed", 1000);
    private final LoggedDashboardNumber ejectSpeed = new LoggedDashboardNumber("Eject Speed", -1000);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            // Real robot, instantiate hardware IO implementations
            case REAL -> {
                drive = new Drive(new DriveIOFalcon500());
                intake = new Intake(new IntakeIOSparkMax());
                arm = new Arm(new ArmIOSparkMax());
            }

            // drive = new Drive(new DriveIOFalcon500());
            // Sim robot, instantiate physics sim IO implementations
            case SIM -> {
                drive = new Drive(new DriveIOSim());
                intake = new Intake(new IntakeIOSparkMax());
                arm = new Arm(new ArmIOSparkMax());
            }

            // Replayed robot, disable IO implementations
            default -> {
                drive = new Drive(new DriveIO() {
                });
                intake = new Intake(new IntakeIO() {
                });
                arm = new Arm(new ArmIO() {
                });
            }
        }

        // Configure the button bindings
        configureButtonBindings();

        intake.setDefaultCommand(new RunCommand(
                () -> {
                    intake.setPrimaryIntakeVelocity(0);
                    intake.setSecondaryIntakeVelocity(0);
                }, intake
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
            () -> arm.setArmPosition(pickupAngle.get()), arm)
            .until(() -> Math.abs(arm.getArmPosition() - pickupAngle.get()) < 5)
            .andThen(() -> {
                intake.setPrimaryIntakeVelocity(intakeSpeed.get());
                intake.setSecondaryIntakeVelocity(intakeSpeed.get());
            }, intake, arm)
            .until(intake::isBeamBroken);

    @AutoBuilderAccessible
    Command outtakeCommand = new RunCommand(
            () -> {
                intake.setPrimaryIntakeVelocity(ejectSpeed.get());
                intake.setSecondaryIntakeVelocity(ejectSpeed.get());
            }, intake
    );

    @AutoBuilderAccessible
    Command shootCommand = new RunCommand(
            () -> {
                intake.setPrimaryIntakeVelocity(shootSpeed.get());
                arm.setArmPosition(shootAngle.get());
            }, intake, arm)
            .until(() -> Math.abs(arm.getArmPosition() - shootAngle.get()) < 5
                    && Math.abs(intake.getPrimaryIntakeVelocity() - shootSpeed.get()) < 100)
            .andThen(() -> {
                intake.setSecondaryIntakeVelocity(3000);
                intake.setPrimaryIntakeVelocity(shootSpeed.get());
                arm.setArmPosition(shootAngle.get());
            }, intake, arm);

    @AutoBuilderAccessible
    Command scoreAmp = new RunCommand(
            () -> arm.setArmPosition(ampScoringAngle.get()), arm)
            .until(() -> Math.abs(arm.getArmPosition() - ampScoringAngle.get()) < 5)
            .andThen(() -> {
                intake.setPrimaryIntakeVelocity(ampScoringSpeed.get());
                intake.setSecondaryIntakeVelocity(ampScoringSpeed.get());
            }, intake, arm);


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

        controller.rightTrigger().whileTrue(intakeCommand);
        controller.leftTrigger().whileTrue(outtakeCommand);

        controller.a().whileTrue(scoreAmp);
        controller.b().whileTrue(shootCommand);
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
