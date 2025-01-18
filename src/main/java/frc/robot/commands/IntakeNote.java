// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.RobotMode;
import frc.robot.RobotContainer;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  public IntakeNote() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intake");
    RobotContainer.intake.mode = RobotMode.RANGE;
    RobotContainer.intake.setAngle(-86.81104874446066);
    RobotContainer.intake.setIntake(0.6, 0.35);
    RobotContainer.intake.setWeird(0.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.setIntake(0, 0);
    RobotContainer.intake.setWeird(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.intake.getRange() < 50;
  }
}
