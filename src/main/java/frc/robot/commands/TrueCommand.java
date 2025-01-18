// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.RobotMode;
import frc.robot.RobotContainer;

public class TrueCommand extends Command {
  private Command ampTrue = new InstantCommand(() -> RobotContainer.intake.setAngle(RobotContainer.intake.ampAngle));
  private Command woofTrue = new SequentialCommandGroup(
        new InstantCommand(() -> RobotContainer.intake.setFly(0.8, 0.9)),
        new InstantCommand(() -> RobotContainer.intake.setAngle(-35))
        );

    private Command rangeTrue = new SequentialCommandGroup(
        new InstantCommand(() -> RobotContainer.intake.setFly(0.92, 0.95))
        );

  /** Creates a new TrueCommand. */
  public TrueCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.intake.getMode() == RobotMode.AMP) {
      CommandScheduler.getInstance().schedule(ampTrue);
    } else if (RobotContainer.intake.getMode() == RobotMode.SUBWOOFER) {
      CommandScheduler.getInstance().schedule(woofTrue);
    } else if (RobotContainer.intake.getMode() == RobotMode.RANGE) {
      CommandScheduler.getInstance().schedule(rangeTrue);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
