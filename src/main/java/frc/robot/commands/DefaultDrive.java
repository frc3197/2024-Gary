// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseEstimator;

public class DefaultDrive extends Command {
  /** Creates a new DefaultDrive. */
  private CommandSwerveDrivetrain drivetrain;

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = 1.5 * Math.PI;

  private SwerveRequest.FieldCentric drive;
  private Supplier<Double> xSup;
  private Supplier<Double> ySup;
  private Supplier<Double> rotSup;
  private Supplier<Boolean> autoAlign;

  private PIDController speakerAlignPID;
  private PoseEstimator poseEstimator;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Pose");
  private final DoubleArraySubscriber poseSubscriber = table.getDoubleArrayTopic("robotPose")
      .subscribe(new double[] {});

  public DefaultDrive(CommandSwerveDrivetrain drivetrain, PoseEstimator poseEstimator, SwerveRequest.FieldCentric drive,
      Supplier<Double> xSup,
      Supplier<Double> ySup, Supplier<Double> rotSup, Supplier<Boolean> autoAlign) {
    this.drivetrain = drivetrain;
    this.poseEstimator = poseEstimator;
    this.drive = drive;
    this.xSup = xSup;
    this.ySup = ySup;
    this.rotSup = rotSup;
    this.autoAlign = autoAlign;

    this.speakerAlignPID = new PIDController(0.075, 0, 0.0);

    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("xSup", xSup.get());
    SmartDashboard.putString("Thing", drivetrain.getCurrentCommand().getName());
    SmartDashboard.putNumber("Rot to Speaker", poseEstimator.getRotationToSpeaker(RobotContainer.isRed()));
    // SmartDashboard.putNumber("Rot thingy", (-poseSubscriber.get()[2]));

    double rotValue = rotSup.get();
    double angle = poseSubscriber.get()[2];
    if (autoAlign.get()) {

      RobotContainer.intake.setAngle(poseEstimator.getElevationAngle(RobotContainer.isRed()) - 99
          + (poseEstimator.getDistanceToSpeakerTop(RobotContainer.isRed()) * 1.7));

      if (RobotContainer.isRed()) {
        if (poseSubscriber.get()[1] < TunerConstants.Vision.speakerBlueTranslation.getY()) {
          rotValue = speakerAlignPID
              .calculate(poseSubscriber.get()[2] - (poseEstimator.getRotationToSpeaker(RobotContainer.isRed()))) / 2.5;
        } else {
          rotValue = speakerAlignPID
              .calculate(poseSubscriber.get()[2] - (poseEstimator.getRotationToSpeaker(RobotContainer.isRed()))) / 2.5;
        }
      } else {
        angle = poseSubscriber.get()[2];
        if (poseSubscriber.get()[1] < TunerConstants.Vision.speakerBlueTranslation.getY()) {
          // Source
          if (angle < 0) {
            angle += 360;
          }
          rotValue = speakerAlignPID
              .calculate((angle - 180) + (poseEstimator.getRotationToSpeaker(RobotContainer.isRed()))) / 2.5;
        } else {
          // Amp
          if (angle < 0) {
            angle += 360;
          }
          SmartDashboard.putNumber("Rot thingy", (angle));
          rotValue = speakerAlignPID
              .calculate((angle - 180) + (poseEstimator.getRotationToSpeaker(RobotContainer.isRed()))) / 2.5;
        }
      }

    }
    if (angle < 0) {
      angle += 360;
    }
    SmartDashboard.putNumber("Rot thingy", (angle));

    if (RobotContainer.getAmpAlignButton()) {
      if (RobotContainer.isRed()) {
        rotValue = speakerAlignPID.calculate(poseSubscriber.get()[2] - 90) / 2.5;
      } else {
        rotValue = speakerAlignPID.calculate(poseSubscriber.get()[2] - 90) / 2.5;
      }
    }

    rotValue = MathUtil.clamp(rotValue, -1, 1);

    if (!DriverStation.isAutonomous()) {
      drivetrain.setControl(drive.withVelocityX(ySup.get() * MaxSpeed) // Drive forward with
          // negative Y (forward)
          .withVelocityY(xSup.get() * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(rotValue * MaxAngularRate));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
