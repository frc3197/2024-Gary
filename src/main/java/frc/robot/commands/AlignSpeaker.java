// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseEstimator;

public class AlignSpeaker extends Command {

  private CommandSwerveDrivetrain drive;
  private PoseEstimator poseEstimator;

  private PIDController speakerAlignPID;
  private double rotationVal;
  private SwerveRequest.FieldCentric actualDrive;

  public AlignSpeaker(CommandSwerveDrivetrain drive, PoseEstimator poseEstimator, SwerveRequest.FieldCentric actualDrive) {
    this.drive = drive;
    this.speakerAlignPID = new PIDController(0.15, 0, 0);
    this.poseEstimator = poseEstimator;
    this.rotationVal = 0;
    this.actualDrive = actualDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationVal = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (RobotContainer.isRed()) {
      rotationVal = speakerAlignPID.calculate(drive.getHeading().getDegrees()
          - (poseEstimator.getRotationToSpeaker(RobotContainer.isRed()))) / 1.15;
    } else {
      if(drive.getHeading().getDegrees() > 0) {
      rotationVal = speakerAlignPID.calculate(((drive.getHeading().getDegrees()-180) * 1) - (-poseEstimator.getRotationToSpeaker(false))) / 1.15;
    } else {
        rotationVal = speakerAlignPID.calculate(((drive.getHeading().getDegrees()+180) * 1) - (-poseEstimator.getRotationToSpeaker(false))) / 1.15;
        SmartDashboard.putNumber("Rot Target", (-poseEstimator.getRotationToSpeaker(false)) + 180);
      }
    }

    

    if (Math.abs(rotationVal) < 0.01) {
      rotationVal = 0;
    }

    SmartDashboard.putNumber("Rot Error", (180-drive.getHeading().getDegrees()));

    rotationVal = MathUtil.clamp(rotationVal, -4, 4);

    drive.applyRequest(() -> actualDrive.withVelocityX(0) // Drive forward with
            // negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(rotationVal) // Drive counterclockwise with negative X
                                                                              // (left)
        );

    System.out.println("Ok.");

  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    //return Math.abs(rotationVal) < 0.00;
  }
}