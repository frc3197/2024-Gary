// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.RobotMode;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DetectCancel;
import frc.robot.commands.FalseCommand;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.TrueCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseEstimator;

public class RobotContainer {
  public static Intake intake = new Intake();
  private static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  public static final PoseEstimator poseEstimator = new PoseEstimator(drivetrain);

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = 1.5 * Math.PI;

  public static final CommandXboxController driveController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);

  PowerDistribution pdh = new PowerDistribution();

  public static boolean enabled = false;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.05) // Deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Field-centric
                                                               // driving in open loop
  // private final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new
  // SwerveRequest.PointWheelsAt();


  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        new DefaultDrive(drivetrain, poseEstimator, drive, () -> -driveController.getLeftX(),
            () -> -driveController.getLeftY(), () -> true ? -driveController.getRightX() : driveController.getRightX(), () -> getSpeakerShootButton()));

    driveController.a().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> intake.setAngle(-86.81104874446066)),
            new InstantCommand(() -> intake.setIntake(0.6, 0.35)),
            new InstantCommand(() -> intake.setWeird(0.8)),
            new Feed().raceWith(new DetectCancel()),
            new InstantCommand(() -> intake.setWeird(0.0)),
            new InstantCommand(() -> intake.setIntake(0, 0))).raceWith(new DetectCancel()));

    driveController.b()
        .onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> intake.setFly(0.95, 0.95))))
        .onFalse((new SequentialCommandGroup(new InstantCommand(() -> intake.setFeed(0.0)),
            new InstantCommand(() -> intake.setFly(0.0, 0.0)))));

    driveController.y()
        .onTrue(new SequentialCommandGroup(new InstantCommand(() -> intake.setFeed(0.5)),
            new InstantCommand(() -> intake.setFly(0.95, 0.95))))
        .onFalse((new SequentialCommandGroup(new InstantCommand(() -> intake.setFeed(0.0)))));

    driveController.rightTrigger(0.25).onTrue(
        new TrueCommand().raceWith(new DetectCancel()))
        .onFalse(
            new FalseCommand().raceWith(new DetectCancel()));

    // driveController.x().onTrue(new InstantCommand(() -> intake.setFly(0.7,
    // 0.9))).onFalse(new InstantCommand(() -> intake.setFly(0.0, 0.0)));
    // driveController.rightBumper().onTrue(new InstantCommand(() ->
    // intake.setFeed(1))).onFalse(new InstantCommand(() -> intake.setFeed(0.0)));
    driveController.rightBumper().onTrue(new InstantCommand(() -> RobotContainer.intake.setAngle(-86.81104874446066)));

    // driveController.y().onTrue(new InstantCommand(() ->
    // intake.setAngle(intake.ampAngle)));

    // Zero robot
    driveController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    driveController.x().onTrue(new InstantCommand(() -> intake.setIntake(-0.6, -0.9)))
        .onFalse(new InstantCommand(() -> intake.setIntake(-0.00, -0.)));

    // Manual arm control
    /*
     * driveController.leftTrigger(0.10)
     * .onTrue(new InstantCommand(() ->
     * intake.setArm(driveController.getLeftTriggerAxis())))
     * .onFalse(new InstantCommand(() -> intake.setArm(0)));
     * driveController.rightTrigger(0.10)
     * .onTrue(new InstantCommand(() ->
     * intake.setArm(-driveController.getRightTriggerAxis())))
     * .onFalse(new InstantCommand(() -> intake.setArm(0)));
     */

    operatorController.a().onTrue(new InstantCommand(() -> {
      intake.mode = RobotMode.AMP;
    }));
    operatorController.y().onTrue(new InstantCommand(() -> {
      intake.mode = RobotMode.SUBWOOFER;
    }));
    operatorController.x().onTrue(new InstantCommand(() -> {
      intake.mode = RobotMode.RANGE;
    }));

    operatorController.rightTrigger(0.25).onTrue(new InstantCommand(() -> intake.setFly(0.92, 0.95)));
    operatorController.rightTrigger(0.25).onFalse(new InstantCommand(() -> intake.setFly(0.0, 0.0)));

    operatorController.povUp().onTrue(new InstantCommand(() -> intake.setClimbers(0.8)))
        .onFalse(new InstantCommand(() -> intake.setClimbers(0)));
    operatorController.povUpLeft().onTrue(new InstantCommand(() -> intake.setClimbers(0.8)))
        .onFalse(new InstantCommand(() -> intake.setClimbers(0)));
    operatorController.povUpRight().onTrue(new InstantCommand(() -> intake.setClimbers(0.8)))
        .onFalse(new InstantCommand(() -> intake.setClimbers(0)));

    operatorController.povDown().onTrue(new InstantCommand(() -> intake.setClimbers(-0.8)))
        .onFalse(new InstantCommand(() -> intake.setClimbers(0)));
    operatorController.povDownLeft().onTrue(new InstantCommand(() -> intake.setClimbers(-0.8)))
        .onFalse(new InstantCommand(() -> intake.setClimbers(0)));
    operatorController.povDownRight().onTrue(new InstantCommand(() -> intake.setClimbers(-0.8)))
        .onFalse(new InstantCommand(() -> intake.setClimbers(0)));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    pdh.setSwitchableChannel(true);

    configureBindings();
  }

  public static boolean getSpeakerAlignButton() {
    return operatorController.getHID().getRightTriggerAxis() > 0.25 && intake.mode == RobotMode.RANGE;
  }

  public static boolean getSpeakerShootButton() {
    return driveController.getHID().getRightTriggerAxis() > 0.25 && intake.mode == RobotMode.RANGE;
  }

  public static boolean getAmpAlignButton() {
    return intake.mode == RobotMode.AMP && operatorController.getHID().getLeftTriggerAxis() > 0.25;
  }

  public static boolean autoTrackSpeaker = false;

  public static boolean getSpeakerElevationButton() {
    return driveController.getHID().getLeftTriggerAxis() > 0.25 || autoTrackSpeaker;
  }

  public static boolean isRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  //private Command runAuto = drivetrain.getAutoPath("Subwoofer");
  private Command runAuto = drivetrain.getAutoPath("Sneaky");

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      drivetrain.runOnce(() -> drivetrain.seedFieldRelative()),
      runAuto
      );

    /*
    return new SequentialCommandGroup(
        drivetrain.runOnce(() -> drivetrain.seedFieldRelative()),
        new SequentialCommandGroup(
            new InstantCommand(() -> RobotContainer.intake.setFly(0.8, 0.9)),
            new InstantCommand(() -> RobotContainer.intake.setAngle(-35))),
        new WaitCommand(1),
        new SequentialCommandGroup(
            new InstantCommand(() -> RobotContainer.intake.setFeed(1)),
                    new WaitCommand(2),
                    new InstantCommand(() -> RobotContainer.intake.setIntake(0, 0)),
                    new InstantCommand(() -> RobotContainer.intake.setFeed(0)),
                    new InstantCommand(() -> RobotContainer.intake.setFly(0.0, 0.0))

        ),
        drivetrain.applyRequest(() -> drive.withVelocityX(isRed() ? 0.25 * MaxSpeed : -0.25 * MaxSpeed) // Drive forward
                                                                                                        // with
            // negative Y (forward)
            .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X
                                                    // (left)
        ),
        new IntakeNote(),
        new InstantCommand(() -> intake.mode = RobotMode.RANGE),
        drivetrain.applyRequest(() -> drive.withVelocityX(isRed() ? 0.25 * MaxSpeed : -0.25 * MaxSpeed) // Drive forward
                                                                                                        // with
            // negative Y (forward)
            .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X
                                                    // (left)
        ),
        new InstantCommand(() -> intake.setAngle(poseEstimator.getElevationAngle(isRed()))),
        new InstantCommand(() -> RobotContainer.intake.setFly(0.8, 0.9)),
            new InstantCommand(() -> RobotContainer.intake.setAngle(-35)),
        new WaitCommand(1),
            new InstantCommand(() -> RobotContainer.intake.setFeed(1)),
                    new WaitCommand(2),
                    new InstantCommand(() -> RobotContainer.intake.setIntake(0, 0)),
                    new InstantCommand(() -> RobotContainer.intake.setFeed(0)),
                    new InstantCommand(() -> RobotContainer.intake.setFly(0.0, 0.0))
        );*/
  }
}
