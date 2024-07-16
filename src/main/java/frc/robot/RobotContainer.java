// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.RobotMode;
import frc.robot.commands.FalseCommand;
import frc.robot.commands.TrueCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  public static Intake intake = new Intake();
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = 1.5 * Math.PI;

  public static final CommandXboxController driveController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);

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
        drivetrain.applyRequest(() -> drive.withVelocityX(driveController.getLeftY() * MaxSpeed) // Drive forward with
            // negative Y (forward)
            .withVelocityY(driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X
                                                                              // (left)
        ));

    driveController.a().onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> intake.setAngle(-86.81104874446066)),
            new InstantCommand(() -> intake.setIntake(0.4, 0.3)),
            new Feed().withTimeout(4),
            new InstantCommand(() -> intake.setIntake(0, 0))));

    driveController.rightTrigger(0.25).onTrue(
      new TrueCommand())
        .onFalse(
            new FalseCommand()
      );

    driveController.x().onTrue(new InstantCommand(() -> intake.setFly(0.7, 0.9))).onFalse(new InstantCommand(() -> intake.setFly(0.0, 0.0)));
    driveController.rightBumper().onTrue(new InstantCommand(() -> intake.setFeed(1))).onFalse(new InstantCommand(() -> intake.setFeed(0.0)));
    
    driveController.y().onTrue(new InstantCommand(() -> intake.setAngle(intake.ampAngle)));

    // Zero robot
    driveController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // Manual arm control
    /*driveController.leftTrigger(0.10)
        .onTrue(new InstantCommand(() -> intake.setArm(driveController.getLeftTriggerAxis())))
        .onFalse(new InstantCommand(() -> intake.setArm(0)));
    driveController.rightTrigger(0.10)
        .onTrue(new InstantCommand(() -> intake.setArm(-driveController.getRightTriggerAxis())))
        .onFalse(new InstantCommand(() -> intake.setArm(0)));
    */

    operatorController.a().onTrue(new InstantCommand(() -> {intake.mode = RobotMode.AMP;}));
    operatorController.y().onTrue(new InstantCommand(() -> {intake.mode = RobotMode.SUBWOOFER;}));
    
    operatorController.povUp().onTrue(new InstantCommand(() -> intake.setClimbers(0.8))).onFalse(new InstantCommand(() -> intake.setClimbers(0)));
    operatorController.povUpLeft().onTrue(new InstantCommand(() -> intake.setClimbers(0.8))).onFalse(new InstantCommand(() -> intake.setClimbers(0)));
    operatorController.povUpRight().onTrue(new InstantCommand(() -> intake.setClimbers(0.8))).onFalse(new InstantCommand(() -> intake.setClimbers(0)));
    
    operatorController.povDown().onTrue(new InstantCommand(() -> intake.setClimbers(-0.8))).onFalse(new InstantCommand(() -> intake.setClimbers(0)));
    operatorController.povDownLeft().onTrue(new InstantCommand(() -> intake.setClimbers(-0.8))).onFalse(new InstantCommand(() -> intake.setClimbers(0)));
    operatorController.povDownRight().onTrue(new InstantCommand(() -> intake.setClimbers(-0.8))).onFalse(new InstantCommand(() -> intake.setClimbers(0)));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
