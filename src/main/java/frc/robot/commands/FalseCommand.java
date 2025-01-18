// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.RobotMode;
import frc.robot.RobotContainer;

public class FalseCommand extends Command {

  private Command ampFalse = new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.intake.setFeed(1)),
      new InstantCommand(() -> RobotContainer.intake.setIntake(-0.4, -0.4)),
      new WaitCommand(1),
      new InstantCommand(() -> RobotContainer.intake.setFeed(0)),
      new InstantCommand(() -> RobotContainer.intake.setIntake(0, 0)),
      new WaitCommand(0));
  private Command woofFalse = new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.intake.setFeed(1)),
      new WaitCommand(2),
      new InstantCommand(() -> RobotContainer.intake.setIntake(0, 0)),
      new InstantCommand(() -> RobotContainer.intake.setFeed(0)),
      new InstantCommand(() -> RobotContainer.intake.setFly(0.0, 0.0)));

  private Command rangeFalse = new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.intake.setFeed(1)),
      new WaitCommand(1),
      new InstantCommand(() -> RobotContainer.intake.setIntake(0, 0)),
      new InstantCommand(() -> RobotContainer.intake.setFeed(0)),
      new InstantCommand(() -> RobotContainer.intake.setFly(0.0, 0.0)));
      //new InstantCommand(() -> RobotContainer.intake.setAngle(-86.81104874446066)));

  /** Creates a new FalseCommand. */
  public FalseCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.intake.getMode() == RobotMode.AMP) {
      CommandScheduler.getInstance().schedule(ampFalse);
    } else if (RobotContainer.intake.getMode() == RobotMode.SUBWOOFER) {
      CommandScheduler.getInstance().schedule(woofFalse);
    } else if (RobotContainer.intake.getMode() == RobotMode.RANGE) {
      CommandScheduler.getInstance().schedule(rangeFalse);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
