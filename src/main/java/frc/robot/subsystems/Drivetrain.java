// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.AprilTagFieldLayouts.*;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.CANConstants.*;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

import java.util.ArrayList;
import java.util.Optional;

import org.apache.commons.lang3.ObjectUtils.Null;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class Drivetrain extends SubsystemBase {
  /** Creates a new SwerveSystem. */
  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;
  public PhotonPipelineResult results;

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(kModuleXOffsetMeters, kModuleYOffsetMeters),
    new Translation2d(kModuleXOffsetMeters, -kModuleYOffsetMeters),
    new Translation2d(-kModuleXOffsetMeters, kModuleYOffsetMeters),
    new Translation2d(-kModuleXOffsetMeters, -kModuleYOffsetMeters)
  );

  private SwerveDriveOdometry odometer;
  private AHRS navx = new AHRS();

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;

  public TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.MeasurementConstants.kMaxSpeedMetersPerSecond/2,
            Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared/2)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(m_kinematics);
      
  public PhotonCamera photonCamera = new PhotonCamera("gloworm");
  public RobotPoseEstimator poseEstimator;
  public Drivetrain() {
    var robotToCam = new Transform3d(new Translation3d(kCameraOffsetX, kCameraOffsetY, kCameraOffsetZ), new Rotation3d(0, 0, 0));
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera,Transform3d>(photonCamera, robotToCam));
    poseEstimator = new RobotPoseEstimator(AprilTagFullFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
    
    
    m_frontLeft = new SwerveModule(
      kFrontLeftDriveMotorID, 
      kFrontLeftSteerMotorID, 
      kFrontLeftEncoderID, 
      kFrontLeftEncoderOffset
    );
    
    m_frontRight = new SwerveModule(
      kFrontRightDriveMotorID, 
      kFrontRightSteerMotorID, 
      kFrontRightEncoderID, 
      kFrontRightEncoderOffset
    );

    m_backLeft = new SwerveModule(
      kBackLeftDriveMotorID, 
      kBackLeftSteerMotorID, 
      kBackLeftEncoderID, 
      kBackLeftEncoderOffset
    );
    
    m_backRight = new SwerveModule(
      kBackRightDriveMotorID, 
      kBackRightSteerMotorID, 
      kBackRightEncoderID, 
      kBackRightEncoderOffset
    );
    
    odometer = new SwerveDriveOdometry(m_kinematics, getGyroRotation2d(), getModulePositions());
  }

  public void recalibrateModulesEncoders() { // Call if modules are not in the correct position
    m_frontLeft.recalibrateRelativeEncoder();
    m_frontRight.recalibrateRelativeEncoder();
    m_backLeft.recalibrateRelativeEncoder();
    m_backRight.recalibrateRelativeEncoder();
  }

  public double getNavxYaw() {
    var pos = navx.getYaw() % 360;
    return pos < -180 ? pos + 360 : pos;
  }

  public double getNavxPitch() {
    return navx.getPitch();
  }

  public double getNavxRoll() {
    return navx.getRoll();
  }

  public void zeroGyro() {
    navx.reset();
  }

  public Rotation2d getGyroRotation2d() {
    return navx.getRotation2d();
  }

  public void updateOdometry() {
    odometer.update(
      getGyroRotation2d(),
      getModulePositions()
    );
  }

  public double getOdometryYaw() {
    var pos = -odometer.getPoseMeters().getRotation().getDegrees() % 360;
    return pos < -180 ? pos + 360 : pos;
  }

  public void zeroOdometry() {
    odometer.resetPosition(getGyroRotation2d(), getModulePositions(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public Pose2d setOdometry(Pose2d pose) {
    var errorX = odometer.getPoseMeters().getX() - pose.getX();
    var errorY = odometer.getPoseMeters().getY() - pose.getY();
    var error = new Pose2d(errorX, errorY, new Rotation2d());
    odometer.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
    return error;
  }

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }
  
  public void setFieldPosition(Pose2d pose) {
    odometer.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
  }

  public Pose2d getFieldPosition() {
    return odometer.getPoseMeters();
  }

  public void stop(){
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    SwerveModuleState[] swerveModuleStates =
      m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed,
          ySpeed,
          rot,
          odometer.getPoseMeters().getRotation()
        )
      );
    
    setModuleStates(swerveModuleStates);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
      m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, odometer.getPoseMeters().getRotation()) : new ChassisSpeeds(xSpeed, ySpeed, rot)
      );
    
    setModuleStates(swerveModuleStates);
  }

  public void printModuleAbsoluteAngles() {
    System.out.println("Front Left: " + m_frontLeft.getAbsoluteAngle());
    System.out.println("Front Right: " + m_frontRight.getAbsoluteAngle());
    System.out.println("Back Left: " + m_backLeft.getAbsoluteAngle());
    System.out.println("Back Right: " + m_backRight.getAbsoluteAngle());
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  
  @Override
  public void periodic() {
    updateOdometry();
    SmartDashboard.putString("Position", odometer.getPoseMeters().toString());
    results = photonCamera.getLatestResult();
  }
  
  public Command getCommandForTrajectory(PathPlannerTrajectory trajectory) {
    xController = new PIDController(kDriveP, 0, 0);
    yController = new PIDController(kDriveP, 0, 0);
    thetaController = new PIDController(kTurnP, 0, 0); //Kp value, Ki=0, Kd=0
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    setFieldPosition(trajectory.getInitialHolonomicPose());

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
      trajectory,
      this::getFieldPosition,
      m_kinematics,
      xController,
      yController,
      thetaController,
      this::setModuleStates,
      this);
    return swerveControllerCommand.andThen(() -> stop());
  }

  //Plan a path to node
  public PathPlannerTrajectory getTrajectoryToPoint(Integer node) {
    SmartDashboard.putString("Error", setOdometry(getRobotPoisitionFromTag(results)).toString());
    var bestTagId = results.getBestTarget().getFiducialId();
    var tagPose = AprilTagList.get(bestTagId).pose.toPose2d();
    var pos = getFieldPosition();
    var offsetX = 0.0;
    // tagPose = Constants.AprilTagFieldLayouts.TagId1.pose.toPose2d();
    if (node == 1) {
      offsetX = -kNodeOffset;
    } else if (node == 3) {
      offsetX = kNodeOffset;
    } else {
      offsetX = 0;
    }

    PathPlannerTrajectory trajectoryToNode = PathPlanner.generatePath(
      new PathConstraints(4, 3), 
      new PathPoint(pos.getTranslation(), Rotation2d.fromDegrees(getNavxPitch()), Rotation2d.fromDegrees((getNavxPitch()))), // position, heading(direction of travel), holonomic rotation
      new PathPoint(new Translation2d(tagPose.getX() + offsetX, tagPose.getY() -kHybridNodeDepth), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(180))); // position, heading(direction of travel), holonomic rotation
    return trajectoryToNode;
  }

  ///**********VISION SECTION *************/
  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = poseEstimator.update();
    if (result.isPresent()) {
        return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
        return new Pair<Pose2d, Double>(null, 0.0);
    }
  }

  public Pose2d getRobotPoisitionFromTag( PhotonPipelineResult photonResult ) {
    var tag = photonResult.targets.get(0);
    Pose3d tagPose = AprilTagList.get(tag.getFiducialId() - 1).pose;
    Transform3d targetToCamera = tag.getBestCameraToTarget().inverse();
    Pose3d camPose = tagPose.transformBy(targetToCamera);
    Pose2d robotPose = new Pose3d(camPose.getX() - kCameraOffsetX, camPose.getY() - kCameraOffsetY, camPose.getZ() - kCameraOffsetZ, camPose.getRotation()).toPose2d();

    return robotPose;
  }
}
