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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

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
      
  // public RobotPoseEstimator poseEstimator;
  public Drivetrain() {
    
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
  }
  
  public void updateOdometryIfTag() {
    if (results.hasTargets() && results.getBestTarget().getFiducialId() < 9) {
      setOdometry(getRobotPoseFromAprilTag());
    }
  }



  ///**********VISION SECTION *************/

  public Pose2d getRobotPoseFromAprilTag() {
    return limelightTable.getEntry("botPose").getValue();
  }

}
