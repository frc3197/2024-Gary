// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Timer;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.RobotMode;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX intakeMotor;
  private TalonFX feedMotor;
  private TalonFX armMotor;
  private TalonFX leftFly;
  private TalonFX rightFly;
  private TalonFX leftClimber;
  private TalonFX rightClimber;
  private TalonFX weirdMotor;
  private TimeOfFlight shooterSensor;
  private TimeOfFlight intakeSensor;
  private ArmFeedforward armFeedforward;
  private DutyCycleEncoder armEncoder;
  private AddressableLED led;
  private AddressableLEDBuffer buffer;

  private SequentialCommandGroup group;
  public boolean intaking = false;

  public RobotMode mode = RobotMode.AMP;

  public double ampAngle = 50;
  private double targetAngle;
  private PIDController pid = new PIDController(0.5, 0, 0);

  private boolean inLab = true;

  public boolean COOL_ALLIANCE_COLORS = true;

  private edu.wpi.first.wpilibj.Timer timer = new edu.wpi.first.wpilibj.Timer();

  public Intake() {
    intakeMotor = new TalonFX(9);
    feedMotor = new TalonFX(11);
    armMotor = new TalonFX(10);
    leftFly = new TalonFX(13);
    rightFly = new TalonFX(12);
    leftClimber = new TalonFX(27);
    rightClimber = new TalonFX(28);
    weirdMotor = new TalonFX(16);
    shooterSensor = new TimeOfFlight(0);
    shooterSensor.setRangingMode(RangingMode.Short, 1);
    intakeSensor = new TimeOfFlight(1);
    intakeSensor.setRangingMode(RangingMode.Short, 1);
    // armFeedforward = new ArmFeedforward(0.0, 0.38, 2.29, 0.01);
    armFeedforward = new ArmFeedforward(0.0, 0.0275, 0.0, 0.00);
    armEncoder = new DutyCycleEncoder(0);
    led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(60 + 102);
    led.setLength(buffer.getLength());

    // Set the data
    led.setData(buffer);
    led.start();
    targetAngle = getArmAngle();

    timer.start();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("TOF", shooterSensor.getRange());
    SmartDashboard.putNumber("Duty", armEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm Angle", getArmAngle());
    SmartDashboard.putNumber("Arm Speed", armMotor.getVelocity().getValueAsDouble());

    if (RobotContainer.getSpeakerAlignButton()) {
      setAngle(RobotContainer.poseEstimator.getElevationAngle(RobotContainer.isRed()) - 98
          + (RobotContainer.poseEstimator.getDistanceToSpeakerTop(RobotContainer.isRed()) * 1.7));
    }

    /*
     * if (RobotContainer.driveController.getHID().getLeftTriggerAxis() < 0.1
     * && RobotContainer.driveController.getHID().getRightTriggerAxis() < 0.1) {
     * setArm(armFeedforward.calculate(Units.degreesToRadians(getArmAngle()),
     * getArmSpeedRadians(armMotor.getVelocity().getValueAsDouble()),
     * 0));
     * }
     */

    setArm((pid.calculate(getArmAngle() - targetAngle) / 20) + armFeedforward.calculate(
        Units.degreesToRadians(getArmAngle()), getArmSpeedRadians(armMotor.getVelocity().getValueAsDouble()), 0));

    SmartDashboard.putNumber("Time", DriverStation.getMatchTime());

    if (!RobotContainer.enabled) {
      if (COOL_ALLIANCE_COLORS) {
        setIdleAlliance();
      } else {
        setIdleColor(65, 255, 0);
      }
    } else if (intakeSensor.getRange() < 95 || timer.get() < 2) {
      if (intaking && intakeSensor.getRange() < 95) {
        timer.reset();
      }
      if (timer.get() % 0.15 < 0.075) {
        setColor(255, 0, 0);
      } else {
        setColor(0, 0, 0);
      }
    } else if (intaking) {
      setColor(100, 100, 100);
    } else if (mode == RobotMode.AMP) {
      setColor(0, 0, 255);
    } else if (mode == RobotMode.RANGE) {
      if (RobotContainer.getSpeakerAlignButton() || RobotContainer.getSpeakerShootButton()) {
        setFlywheelColor(255, 255, 0);
      } else {
        setColor(0, 255, 255);
      }
    } else if (mode == RobotMode.SUBWOOFER) {
      setColor(255, 255, 0);
    }
  }

  public RobotMode getMode() {
    return mode;
  }

  public void setAngle(double angle) {
    targetAngle = angle;
  }

  public void setClimbers(double value) {
    leftClimber.set(value);
    rightClimber.set(value);
  }

  public void setArm(double val) {
    armMotor.set(MathUtil.clamp(val, -0.35, 0.75));
  }

  public void setIntake(double val1, double val2) {
    intakeMotor.set(-val1);
    feedMotor.set(-val2);
    if (val1 == 0.0 || val1 == 0) {
      intaking = false;
    } else {
      intaking = true;
    }
  }

  public void setFeed(double val) {
    feedMotor.set(-val);
  }

  public void setWeird(double val) {
    weirdMotor.set(val);
  }

  public void setFly(double val1, double val2) {
    leftFly.set(val1);
    rightFly.set(-val2);
  }

  public double getRange() {
    return shooterSensor.getRange();
  }

  private double getArmAngle() {

    return (1 - Math.abs(armEncoder.getAbsolutePosition() - 0.1960440049011)
        / Math.abs(0.602667140066679 - 0.1960440049011))
        * (63 - -79) + -79;
  }

  private double getArmSpeedRadians(double value) {
    return (value / 120) * 2 * Math.PI;
  }

  public void setColor(int g, int r, int b) {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      buffer.setRGB(i, g / 4, r / 4, b / 4);
    }
    led.setData(buffer);
  }

  public void setFlywheelColor(int r, int g, int b) {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      if ((i + Math.round(timer.get() * 4)) % 6 * (Math.random()) >= 3) {
        buffer.setRGB(i, r, g, b);
      } else {
        buffer.setRGB(i, 0, 0, 0);
      }
    }
    led.setData(buffer);
  }

  public void setIdleColor(int r, int g, int b) {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      if ((i + Math.round(timer.get() * 4)) % 6 >= 3) {
        buffer.setRGB(i, r / 4, g / 4, b / 4);
      } else {
        buffer.setRGB(i, 0, 0, 0);
      }
    }
    led.setData(buffer);
  }

  public void setIdleAlliance() {
    SmartDashboard.putNumber("COOL", (int)Math.cos((timer.get())) * 100.0);
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      if (RobotContainer.isRed())
        buffer.setRGB(i, 0, 130 + (int)Math.round(Math.cos((timer.get()*1.25 + (i / 6)) * 2.75) * 125.0), 0);
      else
        buffer.setRGB(i, 0, 0, 130 + (int)Math.round(Math.cos((timer.get()*1.25 + (i / 6)) * 2.75) * 125.0));
    }
    led.setData(buffer);
  }

  public Command subwooferShootCommand = new SequentialCommandGroup(
      new InstantCommand(() -> RobotContainer.intake.setFly(0.8, 0.9)),
      new InstantCommand(() -> RobotContainer.intake.setAngle(-35)),
      new WaitCommand(0.5),
      new InstantCommand(() -> RobotContainer.intake.setFeed(1)),
      new WaitCommand(0.35),
      new InstantCommand(() -> RobotContainer.intake.setIntake(0, 0)),
      new InstantCommand(() -> RobotContainer.intake.setFeed(0)),
      new InstantCommand(() -> RobotContainer.intake.setFly(0.0, 0.0)));
}
