// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.SwerveModuleConstants.*;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;
import static frc.robot.Constants.MeasurementConstants.*;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriveWithJoysticks extends CommandBase {

  Drivetrain m_drivetrain;

  DoubleSupplier m_x;
  DoubleSupplier m_y;
  DoubleSupplier m_theta;
  DoubleSupplier m_precision;
  Trigger m_faceForwards;
  
  boolean m_PIDcontrol;
  
  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);
  
  double m_toAngle;
  double m_xSpeed;
  double m_ySpeed;
  double m_thetaSpeed;
  double m_precisionFactor;

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private final SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  /** Creates a new Drive. */
  public DriveWithJoysticks(
      Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, DoubleSupplier precision, Trigger faceForwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;

    m_faceForwards = faceForwards;

    m_x = x;
    m_y = y;
    m_theta = theta;
    m_precision = precision;

    turnController.enableContinuousInput(-180, 180);

    addRequirements(drivetrain);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_toAngle = 0;
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_faceForwards.getAsBoolean()) {
      m_PIDcontrol = true;
    } else {
      m_PIDcontrol = false;
    }

    m_precisionFactor = Math.pow(0.4, m_precision.getAsDouble());
    m_xSpeed =
      -m_xLimiter.calculate(MathUtil.applyDeadband(m_y.getAsDouble(), kDriveDeadband))
      * kMaxSpeedMetersPerSecond * kSpeedMultiplier * m_precisionFactor;
    
    m_ySpeed =
      -m_yLimiter.calculate(MathUtil.applyDeadband(m_x.getAsDouble(), kDriveDeadband))
      * kMaxSpeedMetersPerSecond * kSpeedMultiplier * m_precisionFactor;

    if (DriverStation.getAlliance() == Alliance.Blue) {
      m_ySpeed = -m_ySpeed;
      m_xSpeed = -m_xSpeed;
    }
    
    if(m_PIDcontrol) {
      // PID control
      turnController.setSetpoint(m_toAngle);
      m_thetaSpeed = -turnController.calculate(m_drivetrain.getOdometryYaw());
      m_thetaSpeed = MathUtil.clamp(m_thetaSpeed, -kMaxAngularSpeedRadiansPerSecond * kSpeedMultiplier, kMaxAngularSpeedRadiansPerSecond * kSpeedMultiplier);
    } else {
      // Joystick control
      m_thetaSpeed =
        -m_thetaLimiter.calculate(MathUtil.applyDeadband(m_theta.getAsDouble(), kDriveDeadband))
        * kMaxAngularSpeedRadiansPerSecond * kSpeedMultiplier * m_precisionFactor;
    }

    m_drivetrain.drive(m_xSpeed, m_ySpeed, m_thetaSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}