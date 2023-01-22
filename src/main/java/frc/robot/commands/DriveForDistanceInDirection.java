// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

public class DriveForDistanceInDirection extends CommandBase {
  Pose2d currentPos;
  Drivetrain m_drivetrain;
  Double xDistance, yDistance;
  private PIDController yController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController xController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController turnController = new PIDController(0.05, 0, 0);
  
  public DriveForDistanceInDirection(Drivetrain drivetrain, Double x, Double y) {
    m_drivetrain = drivetrain;
    xDistance = x;
    yDistance = y; // 2.3 m
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(2);
    xController.setTolerance(0.01);
    yController.setTolerance(0.01);
    currentPos = m_drivetrain.getFieldPosition();
    xController.setSetpoint(currentPos.getX() + xDistance);
    yController.setSetpoint(currentPos.getY() + yDistance);
    SmartDashboard.putString("StartingPos", currentPos.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xDrive = xController.calculate(m_drivetrain.getFieldPosition().getX());
    var yDrive = yController.calculate(m_drivetrain.getFieldPosition().getY());
    var rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees());
    rot = MathUtil.clamp(rot, -1, 1);
    xDrive = MathUtil.clamp(xDrive, -1.4, 1.4);
    yDrive = MathUtil.clamp(yDrive, -1.4, 1.4);
    m_drivetrain.drive(xDrive, yDrive, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Ending Pos", m_drivetrain.getFieldPosition().toString());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xController.atSetpoint() && yController.atSetpoint()) {
      return true;
    }
    return false;
  }
}
