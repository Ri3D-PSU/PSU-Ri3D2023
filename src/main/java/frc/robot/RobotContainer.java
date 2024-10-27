// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOFalcon500;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import java.util.Set;
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
    private Climber climber;
    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<String> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    private final LoggedDashboardChooser<String> sideChooser = new LoggedDashboardChooser<>("Side Chooser");

    private final LoggedDashboardNumber shootSpeed = new LoggedDashboardNumber("Shoot Speed", -1500.0);
    private final LoggedDashboardNumber shootAngle = new LoggedDashboardNumber("Shoot Angle", 120);
    private final LoggedDashboardNumber pickupAngle = new LoggedDashboardNumber("Pickup Angle", -11);
    private final LoggedDashboardNumber ampScoringAngle = new LoggedDashboardNumber("Amp Scoring Angle", 123);
    private final LoggedDashboardNumber ampScoringSpeed = new LoggedDashboardNumber("Amp Scoring Speed", 1000.0);
    private final LoggedDashboardNumber intakeSpeed = new LoggedDashboardNumber("Intake Speed", 1000);
    private final LoggedDashboardNumber ejectSpeed = new LoggedDashboardNumber("Eject Speed", -1000);
    private final LoggedDashboardNumber sourcePickupAngle = new LoggedDashboardNumber("Source Pickup Angle", 70);
    private final LoggedDashboardNumber sourcePickupSpeed = new LoggedDashboardNumber("Source Pickup Speed", -1000);


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
                climber = new Climber(new ClimberIOSparkMax());
            }

            // drive = new Drive(new DriveIOFalcon500());
            // Sim robot, instantiate physics sim IO implementations
            case SIM -> {
                drive = new Drive(new DriveIOSim());
                intake = new Intake(new IntakeIOSparkMax());
                arm = new Arm(new ArmIOSparkMax());
                climber = new Climber(new ClimberIOSparkMax());
            }

            // Replayed robot, disable IO implementations
            default -> {
                drive = new Drive(new DriveIO() {
                });
                intake = new Intake(new IntakeIO() {
                });
                arm = new Arm(new ArmIO() {
                });
                climber = new Climber(new ClimberIO() {
                });
            }
        }

        intake.setDefaultCommand(new RunCommand(
                () -> {
                    intake.setPrimaryIntakeVelocity(0);
                    intake.setSecondaryIntakeVelocity(0);
                }, intake
        ));

        intakeCommand = new RunCommand(
                () -> {
                    intake.setPrimaryIntakeVelocity(intakeSpeed.get());
                    intake.setSecondaryIntakeVelocity(intakeSpeed.get());
                }, intake)
                .withTimeout(0.25)
                .andThen(new RunCommand(() -> {
                            intake.setPrimaryIntakeVelocity(intakeSpeed.get());
                            intake.setSecondaryIntakeVelocity(intakeSpeed.get());
                        }, intake)
                                .until(() -> intake.getSecondaryIntakeCurrent() > 6)
                );

        setArmIntake = new RunCommand(
                () -> arm.setArmPositionDegrees(pickupAngle.get()), arm);

        outtakeCommand = new RunCommand(
                () -> {
                    intake.setPrimaryIntakeVelocity(ejectSpeed.get());
                    intake.setSecondaryIntakeVelocity(ejectSpeed.get());
                }, intake);

        setArmShoot = new RunCommand(
                () -> arm.setArmPositionDegrees(shootAngle.get()), arm);
        shootCommand = new RunCommand(
                () -> {
                    intake.setPrimaryIntakeVelocity(shootSpeed.get());
                }, intake)
                .withTimeout(1)
                .andThen(new RunCommand(() -> {
                    intake.setSecondaryIntakeVelocity(-3000);
                    intake.setPrimaryIntakeVelocity(shootSpeed.get());
                }, intake));

        scoreAmp = new Command() {

            double timeLeftEjecting = 0;
            double stalledTime = 0;

            @Override
            public void initialize() {
                timeLeftEjecting = 0;
                stalledTime = 0;
            }

            @Override
            public void execute() {
                timeLeftEjecting -= 0.02;

                if (timeLeftEjecting <= 0 && intake.getPrimaryIntakeCurrent() > 35) {
                    stalledTime += 0.02;
                } else {
                    stalledTime = 0;
                }

                if (stalledTime > 0.2) {
                    timeLeftEjecting = 0.05;
                }

                if (timeLeftEjecting <= 0) {
                    intake.setPrimaryIntakeVelocity(ampScoringSpeed.get());
                    intake.setSecondaryIntakeVelocity(ampScoringSpeed.get());
                } else {
                    intake.setPrimaryIntakeVelocity(-200);
                    intake.setSecondaryIntakeVelocity(-200);
                }
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(intake);
            }
        };

        setArmAmp = new RunCommand(
                () -> arm.setArmPositionDegrees(ampScoringAngle.get()), arm);

        climberDownCommand = new StartEndCommand(
                () -> climber.setClimberPower(-0.5), () -> climber.stopClimber(), climber);

        climberUpCommand = new StartEndCommand(
                () -> climber.setClimberPower(0.8), () -> climber.stopClimber(), climber);

        climberMainDownCommand = new StartEndCommand(
                () -> climber.setMainClimberPower(-0.2), () -> climber.stopClimber(), climber);

        climberMainUpCommand = new StartEndCommand(
                () -> climber.setMainClimberPower(0.2), () -> climber.stopClimber(), climber);

        stopClimbCommand = new RunCommand(
                () -> climber.stopClimber(), climber);

        deployClimberCommand = new StartEndCommand(
                () -> climber.extendClimber(), () -> climber.setClimberPower(0), climber);

        setArmSource = new RunCommand(() -> arm.setArmPositionDegrees(sourcePickupAngle.get()), arm);

        // Configure the button bindings
        configureButtonBindings();

        arm.setDefaultCommand(setArmIntake);

        // Initialize autonomous container
        // AutonomousContainer.getInstance().setDebugPrints(true);
        // AutonomousContainer.getInstance().initialize(
        //         false,
        //         info -> new RamseteCommand(
        //                 info.trajectory(),
        //                 drive::getPose,
        //                 new RamseteController(),
        //                 drive.getFeedforward(),
        //                 drive.getKinematics(),
        //                 drive::getWheelSpeeds,
        //                 new PIDController(0.5, 0, 0),
        //                 new PIDController(0.5, 0, 0),
        //                 drive::driveVoltage,
        //                 drive
        //         ),
        //         drive::resetOdometry,
        //         true,
        //         this
        // );

        // Set up auto routines
        sideChooser.addDefaultOption("Blue", "blue");
        sideChooser.addOption("Red", "red");

        // AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));
    }


    Command intakeCommand;
    Command setArmIntake;
    Command outtakeCommand;
    Command shootCommand;
    Command setArmShoot;
    Command scoreAmp;
    Command setArmAmp;
    Command climberDownCommand;
    Command climberUpCommand;
    Command climberMainDownCommand;
    Command climberMainUpCommand;
    Command stopClimbCommand;
    Command deployClimberCommand;
    Command setArmSource;

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drive.setDefaultCommand(
                new RunCommand(() -> drive.driveArcade(-controller.getLeftY(), -controller.getRightX() / 1.5), drive));
        controller.leftBumper()
                .whileTrue(new RunCommand(
                        () -> drive.driveArcade(-controller.getLeftY() / 2.0, -controller.getRightX() / 2.0),
                        drive));

        controller.leftTrigger().whileTrue(outtakeCommand);

        controller.a().onTrue(setArmIntake);
        controller.b().onTrue(setArmSource);
        controller.y().onTrue(setArmAmp);

        // controller.rightTrigger().and(setArmIntake::isScheduled).whileTrue(intakeCommand);
        controller.rightTrigger().and(setArmSource::isScheduled).whileTrue(intakeCommand);
        controller.rightTrigger().and(setArmAmp::isScheduled).whileTrue(scoreAmp);

        controller.povUp().and(() -> !controller.start().getAsBoolean()).whileTrue(climberUpCommand);
        controller.povDown().and(() -> !controller.start().getAsBoolean()).whileTrue(climberDownCommand);
        
        controller.povUp().and(() -> controller.start().getAsBoolean()).whileTrue(climberMainUpCommand);
        controller.povDown().and(() -> controller.start().getAsBoolean()).whileTrue(climberMainDownCommand);

        controller.rightBumper().whileTrue(shootCommand);

        controller.povLeft().onTrue(new InstantCommand(() -> arm.adjustAnglePosition(-5)));
        controller.povRight().onTrue(new InstantCommand(() -> arm.adjustAnglePosition(5)));

    }


    public void setArmAngle(double degrees) {
        arm.resetAngle(Math.toRadians(degrees));
    }


}
