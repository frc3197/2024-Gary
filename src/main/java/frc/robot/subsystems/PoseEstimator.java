// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
/*
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
*/

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class PoseEstimator extends SubsystemBase {

  // Pose estimator standard deviations
  private static final edu.wpi.first.math.Vector<N3> stateStdDevs = VecBuilder.fill(0.025, 0.025,
      Units.degreesToRadians(0.05));
  private static final edu.wpi.first.math.Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.2, 0.2,
      Units.degreesToRadians(5));

  // Pose estimator, field image
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d;

  // Drive subsystem
  private CommandSwerveDrivetrain drive;

  // Limelight
  private NetworkTable limelightNetworkTable;
  private double cameraLatency;

  private AprilTagFieldLayout aprilTagFieldLayout;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Pose");
  private final DoubleArraySubscriber poseSubscriber = table.getDoubleArrayTopic("robotPose")
      .subscribe(new double[] {});

  //private PhotonCamera intake = new PhotonCamera("USB_Camera");

  public PoseEstimator(CommandSwerveDrivetrain drive) {
    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    this.drive = drive;

    field2d = new Field2d();

    ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

     poseEstimator = new SwerveDrivePoseEstimator(
        TunerConstants.swerveKinematics,
        this.drive.getPigeon2().getRotation2d(),
        getPositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);

    visionTab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    visionTab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);

    limelightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight-front");

  }

  private SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = {this.drive.getModule(0).getPosition(false), this.drive.getModule(1).getPosition(false), this.drive.getModule(2).getPosition(false), this.drive.getModule(3).getPosition(false)};

    return positions;
  }

  @Override
  public void periodic() {

    Pose3d bestPose = getDesiredPose();

    if (bestPose != null && !DriverStation.isAutonomous())
      drive.addVisionMeasurement(bestPose.toPose2d(), Timer.getFPGATimestamp() - cameraLatency);

    poseEstimator.update(
        this.drive.getPigeon2().getRotation2d(),
        getPositions());

    field2d.setRobotPose(getCurrentPose());

    SmartDashboard.putNumber("Rotation to speaker", getRotationToSpeaker(RobotContainer.isRed()));
    SmartDashboard.putNumber("X to speaker", getXDistanceToRedSpeaker());
    SmartDashboard.putNumber("Y to speaker", getYDistanceToRedSpeaker());
    SmartDashboard.putNumber("Gyro Heading", this.drive.getPigeon2().getAngle());
    
    SmartDashboard.putNumber("Elevation to speaker", getElevationAngle(RobotContainer.isRed()));
    SmartDashboard.putNumber("Distance to speaker", getDistanceToRedSpeaker());

    SmartDashboard.putBoolean("Red", RobotContainer.isRed());

  }

  public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(this.drive.getPigeon2().getRotation2d(), getPositions(), pose);
  }

  // Returns pose to be used in Shuffleboard
  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  // Returns pose estimator pose
  public Pose2d getCurrentPose() {
    return new Pose2d(new Translation2d(poseSubscriber.get()[0], poseSubscriber.get()[1]), new Rotation2d(poseSubscriber.get()[2]));
  }

  // Resets pose estimator pose
  public void resetPose() {
    poseEstimator.resetPosition(this.drive.getPigeon2().getRotation2d(), getPositions(), getCurrentPose());
  }

  // Returns the best vision pose, for updating pose estimator
  private Pose3d getDesiredPose() {

    // If limelight tag has enough area, favor it over side cameras
    if (getLimelightArea() > 0) {
      double[] limelightBotPose = getBotPoseBlue();

      cameraLatency = limelightBotPose[6] / 1000.0;

      return new Pose3d(limelightBotPose[0], limelightBotPose[1], limelightBotPose[2],
          new Rotation3d(Units.degreesToRadians(limelightBotPose[3]), Units.degreesToRadians(limelightBotPose[4]),
              Units.degreesToRadians(limelightBotPose[5])));
    }
    return null;
  }

  // Returns limelight tag area
  private double getLimelightArea() {
    return limelightNetworkTable.getEntry("ta").getDouble(0);
  }

  // Returns limelight pose relative to blue origin
  private double[] getBotPoseBlue() {
    return limelightNetworkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
  }

  public void setLimelightLight(int mode) {
    limelightNetworkTable.getEntry("ledMode").setNumber(mode);
  }

  private double getDistanceToRedSpeaker() {
    return Math.sqrt(Math.pow(getCurrentPose().getX() - TunerConstants.Vision.speakerRedTranslation.getX(), 2)
        + Math.pow(getCurrentPose().getY() - TunerConstants.Vision.speakerRedTranslation.getY(), 2));
  }

  private double getDistanceToBlueSpeaker() {
    return Math.sqrt(Math.pow(getCurrentPose().getX() - TunerConstants.Vision.speakerBlueTranslation.getX(), 2)
        + Math.pow(getCurrentPose().getY() - TunerConstants.Vision.speakerBlueTranslation.getY(), 2));
  }

  public double getDistanceToSpeaker() {
    return RobotContainer.isRed() ? getDistanceToRedSpeaker() : getDistanceToBlueSpeaker();
  }

  public double getXDistanceToRedSpeaker() {
    return Math.abs(getCurrentPose().getX() - TunerConstants.Vision.speakerRedTranslation.getX());
  }

  public double getYDistanceToRedSpeaker() {
    return getCurrentPose().getY() - TunerConstants.Vision.speakerRedTranslation.getY();
  }

  private double getXDistanceToBlueSpeaker() {
    return Math.abs(getCurrentPose().getX() - TunerConstants.Vision.speakerBlueTranslation.getX());
  }

  private double getYDistanceToBlueSpeaker() {
    return getCurrentPose().getY() - TunerConstants.Vision.speakerBlueTranslation.getY();
  }

  // Returns in degrees
  public double getElevationAngle(boolean red) {
    return Units.radiansToDegrees(
        Math.atan(TunerConstants.Vision.speakerHeight / (red ? getDistanceToRedSpeaker() : getDistanceToBlueSpeaker())));
  }

  public double getDistanceToSpeakerTop(boolean red) {
    return Math.sqrt(Math.pow(red ? getDistanceToRedSpeaker() : getDistanceToBlueSpeaker(), 2) + Math.pow(TunerConstants.Vision.speakerHeight, 2)); 
  }

  public double getRotationToSpeaker(boolean red) {
    if (getCurrentPose().getY() > TunerConstants.Vision.speakerRedTranslation.getY()) {
      return Math.toDegrees((Math.atan((red ? getXDistanceToRedSpeaker() : getXDistanceToBlueSpeaker())
          / (red ? getYDistanceToRedSpeaker() : getYDistanceToBlueSpeaker())))) - 90;
    }
    return 90 + Math.toDegrees((Math.atan((red ? getXDistanceToRedSpeaker() : getXDistanceToBlueSpeaker())
        / (red ? getYDistanceToRedSpeaker() : getYDistanceToBlueSpeaker()))));
  }

  public double getFormulaAngleToSpeaker(boolean red) {
    return red ? evaluateFormula(getDistanceToBlueSpeaker()) : evaluateFormula(getDistanceToRedSpeaker());
  }

  private double evaluateFormula(double distance) {
    return distance;
  }
}