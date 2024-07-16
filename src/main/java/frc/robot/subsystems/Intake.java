// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private TimeOfFlight shooterSensor;
  private ArmFeedforward armFeedforward;
  private DutyCycleEncoder armEncoder;
  private AddressableLED led;
  private AddressableLEDBuffer buffer;

  private SequentialCommandGroup group; 

  public RobotMode mode = RobotMode.AMP;

  public double ampAngle = 50;
  private double targetAngle;
  private PIDController pid = new PIDController(0.2, 0, 0);

  public Intake() {
    intakeMotor = new TalonFX(9);
    feedMotor = new TalonFX(11);
    armMotor = new TalonFX(10);
    leftFly = new TalonFX(13);
    rightFly = new TalonFX(12);
    leftClimber = new TalonFX(27);
    rightClimber = new TalonFX(28);
    shooterSensor = new TimeOfFlight(0);
    shooterSensor.setRangingMode(RangingMode.Short, 1);
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
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("TOF", shooterSensor.getRange());
    SmartDashboard.putNumber("Duty", armEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm Angle", getArmAngle());
    SmartDashboard.putNumber("Arm Speed", armMotor.getVelocity().getValueAsDouble());

    /*if (RobotContainer.driveController.getHID().getLeftTriggerAxis() < 0.1
        && RobotContainer.driveController.getHID().getRightTriggerAxis() < 0.1) {
      setArm(armFeedforward.calculate(Units.degreesToRadians(getArmAngle()),
          getArmSpeedRadians(armMotor.getVelocity().getValueAsDouble()),
          0));
    }*/

    setArm((pid.calculate(getArmAngle() - targetAngle)/20) + armFeedforward.calculate(Units.degreesToRadians(getArmAngle()), getArmSpeedRadians(armMotor.getVelocity().getValueAsDouble()), 0));
  
    if(mode == RobotMode.AMP) {
      setColor(0, 0, 255);
    } else {
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
    rightClimber.set(-value);
  }

  public void setArm(double val) {
    armMotor.set(MathUtil.clamp(val, -0.8, 0.8));
  }

  public void setIntake(double val1, double val2) {
    intakeMotor.set(-val1);
    feedMotor.set(-val2);
  }

  public void setFeed(double val) {
    feedMotor.set(-val);
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

  public void setColor(int r, int g, int b) {
    for (int i = 0; i < buffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      buffer.setRGB(i, r, g, b);
    }
    led.setData(buffer);
  }
}
