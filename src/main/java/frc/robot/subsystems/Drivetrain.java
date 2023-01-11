// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
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

import org.photonvision.PhotonCamera;
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
  // public RobotPoseEstimator poseEstimator;
  public Drivetrain() {
    var robotToCam = new Transform3d(new Translation3d(kCameraOffsetX, kCameraOffsetY, kCameraOffsetZ), new Rotation3d(0, 0, 0));
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera,Transform3d>(photonCamera, robotToCam));
    // poseEstimator = new RobotPoseEstimator(AprilTagFullFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);
    
    
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
    var pos = navx.getYaw() + Timer.getFPGATimestamp()*0.005  % 360;
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

  public void setOdometry(Pose2d pose) {
    odometer.resetPosition(new Rotation2d(0), getModulePositions(), pose);
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
    SmartDashboard.putNumber("GyroRotation", getNavxYaw());
    SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
    SmartDashboard.putString("Position", odometer.getPoseMeters().toString());
    results = photonCamera.getLatestResult();
  }
  
  public void updateOdometryIfTag() {
    if (results.hasTargets() && results.getBestTarget().getFiducialId() < 9) {
      setOdometry(getRobotPoseFromAprilTag());
    }
  }

  public Command getCommandForTrajectory(PathPlannerTrajectory trajectory) {
    xController = new PIDController(kDriveP, 0, 0);
    yController = new PIDController(kDriveP, 0, 0);
    thetaController = new PIDController(kTurnP, 0, 0); //Kp value, Ki=0, Kd=0
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // setFieldPosition(trajectory.getInitialHolonomicPose());

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
    results = photonCamera.getLatestResult();
    if (!results.hasTargets()) {
      PathPlannerTrajectory noTrajectory = PathPlanner.generatePath(
      new PathConstraints(0.2, 0.1), 
       // position, heading(direction of travel), holonomic rotation
      new PathPoint(odometer.getPoseMeters().getTranslation(), new Rotation2d(-getNavxPitch()), new Rotation2d(-getNavxPitch())),
      new PathPoint(odometer.getPoseMeters().getTranslation(), new Rotation2d(-getNavxPitch()), new Rotation2d(-getNavxPitch())));
      SmartDashboard.putString("NoTrajectoryPose", odometer.getPoseMeters().getTranslation().toString());
      return noTrajectory;
    }
    // SmartDashboard.putString("Error", setOdometry(getEstimatedGlobalPose(getFieldPosition())).toString());
      var bestTagId = results.getBestTarget().getFiducialId();
      var tagPose = AprilTagList.get(bestTagId).pose.toPose2d();
      var pos = getFieldPosition();
      var offsetY = 0.0;
      // tagPose = Constants.AprilTagFieldLayouts.TagId1.pose.toPose2d();
      if (node == 1) {
        offsetY = -kNodeOffset;
      } else if (node == 3) {
        offsetY = kNodeOffset;
      } else {
        offsetY = 0;
      }

    PathPlannerTrajectory trajectoryToNode = PathPlanner.generatePath(
      new PathConstraints(1, 1), 
      new PathPoint(pos.getTranslation(), Rotation2d.fromDegrees(-getNavxPitch()), Rotation2d.fromDegrees((-getNavxPitch()))), // position, heading(direction of travel), holonomic rotation
      new PathPoint(new Translation2d(14, 1.07), Rotation2d.fromDegrees(-getNavxPitch()), Rotation2d.fromDegrees((-getNavxPitch())))
      // new PathPoint(new Translation2d(tagPose.getX() -kHybridNodeDepth - 0.7, tagPose.getY() + offsetY), Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90))); // position, heading(direction of travel), holonomic rotation
      ); // position, heading(direction of travel), holonomic rotation
    return trajectoryToNode;
  }

  ///**********VISION SECTION *************/

  // public Pose2d getRobotPoseFromAprilTag() {
  //   var tag = photonCamera.getLatestResult().getBestTarget();
  //   var tagFieldPos = Constants.AprilTagFieldLayouts.AprilTagList.get(tag.getFiducialId() - 1);
  //   var tagPos = tag.getBestCameraToTarget();
    
  //   var yRobotToCamera = (15 / kInchesToMeters) * Math.sin(-getNavxPitch()*(Math.PI/180));
  //   SmartDashboard.putNumber("offset Y from robot to camera m", yRobotToCamera);
    
  //   var xRobotToCamera = (15 / kInchesToMeters) * Math.cos(-getNavxPitch()*(Math.PI/180));
  //   SmartDashboard.putNumber("offset X from robot to camera m", xRobotToCamera);
    
  //   var hypoteneuse = Math.sqrt( (tagPos.getX() * tagPos.getX()) + (tagPos.getY() * tagPos.getY()));
  //   SmartDashboard.putNumber("Hypoteneuse m", hypoteneuse);

  //   var xRobotToTag = hypoteneuse * Math.cos((getNavxYaw() - tag.getYaw()) * (Math.PI / 180));
  //   SmartDashboard.putNumber("X OFFSET ROBOT TO TAG m", xRobotToTag);
    
  //   var yRobotToTag = Math.sqrt((hypoteneuse * hypoteneuse) - (xRobotToTag * xRobotToTag));
  //   // var yRobotToTag = hypoteneuse * Math.sin((getNavxYaw() - tag.getYaw()) * (Math.PI / 180));

  //   SmartDashboard.putNumber("Y OFFSET ROBOT TO TAG m", yRobotToTag);
  //   var offsetY = 0.0;
  //   if (tag.getYaw() > 0) {
  //     offsetY = yRobotToCamera + yRobotToTag;
  //   } else {
  //     offsetY = yRobotToCamera - yRobotToTag;
  //   }
  //   var offsetX = xRobotToCamera + xRobotToTag;
  //   // var offsetX = (15 / kInchesToMeters) * Math.sin(-getNavxPitch()*(Math.PI/180))
  //   // + Math.sqrt(Math.pow(tagPos.getX(), 2) + Math.pow(tagPos.getY(), 2)) 
  //   // //  * Math.cos(tagRot * Math.PI/180);
  //   // * Math.cos((getNavxYaw() + tag.getYaw()) * (Math.PI / 180));
    
  //   // var offsetY = (15 / kInchesToMeters) * Math.cos(-getNavxPitch()*(Math.PI/180))
  //   // + Math.sqrt(Math.pow(tagPos.getX(), 2) + Math.pow(tagPos.getY(), 2)) 
  //   // // * Math.sin(tagRot * Math.PI/180);
  //   // * Math.sin((getNavxYaw() + tag.getYaw()) * (Math.PI / 180));
  //   if (tag.getFiducialId() < 5) {
  //     offsetX *= -1;
  //     offsetY *= -1;
  //   }
  //   SmartDashboard.putNumber("X OFFSET", offsetX);
  //   SmartDashboard.putNumber("Y OFFSET", offsetY);
  //   var robotPos = new Pose2d(tagFieldPos.pose.getX() + offsetX, tagFieldPos.pose.getY() + offsetY, new Rotation2d(0)/*new Rotation2d(getNavxYaw() * Math.PI / 180)*/);
  //   SmartDashboard.putString("Field Pose", robotPos.toString());
  //   return robotPos;
    


  //   // Camera to AprilTag field Y Offset = sqrt(x2 + y2)Sin()
  //   // Camera to AprilTag field x offset = sqrt(x2 + y2)Cos()
  //   // Camera To Robot field x offset = 15”Sin(-Pitch)
  //   // Camera To Robot field y offset =  15”Cos(-Pitch)
    
  // }

  private Pose2d getRobotPoseFromAprilTag() {
    var tag = results.getBestTarget();
    var tagFieldPos = Constants.AprilTagFieldLayouts.AprilTagList.get(tag.getFiducialId() - 1);
    var tagpos = tag.getBestCameraToTarget();
    var C = tagpos.getY() * Math.tan(getNavxYaw());
    var offsetH = 15 / kInchesToMeters + tagpos.getX() + C;
    var offsetX = offsetH * Math.cos(getNavxYaw());
    var offsetY = offsetH * Math.sin(getNavxYaw()) - Math.sin(getNavxYaw()) / tagpos.getY();
    var robotPose = new Pose2d(tagFieldPos.pose.getX() + offsetX, tagFieldPos.pose.getY() + offsetY, new Rotation2d(0)/*new Rotation2d(getNavxYaw() * Math.PI / 180)*/);
    return robotPose;
  }

}
