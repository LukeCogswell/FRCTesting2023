// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

public class AlignWithNode extends CommandBase {
  /** Creates a new AlignWithNode. */
  private PhotonPipelineResult results;
  private Drivetrain m_drivetrain;
  private Integer node;
  private double targetXPos;
  private double targetYPos;
  private PIDController yController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController xController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController turnController = new PIDController(kSteerP, kSteerI, kSteerD);


  public AlignWithNode(Drivetrain drivetrain, Integer whichNode) {
    node = whichNode;
    m_drivetrain = drivetrain;

    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnController.enableContinuousInput(-180, 180);
    results = m_drivetrain.photonCamera.getLatestResult();
    if (results.hasTargets()) {
    var tag = results.getBestTarget();
    var tagPose = Constants.AprilTagFieldLayouts.AprilTagList.get(tag.getFiducialId() - 1).pose;
    var offsetY = 0.0;
    if (tag.getFiducialId() < 5) {
      xController.setSetpoint(14.5);
      turnController.setSetpoint(0);
      if (node == 3 ) {
        offsetY -= kNodeOffset; 
      } else if (node == 1) {
        offsetY += kNodeOffset;
      }
      targetYPos = tagPose.getY() + offsetY;
    } else {
      xController.setSetpoint(2);
      turnController.setSetpoint(180);
      if (node == 1 ) {
        offsetY -= kNodeOffset; 
      } else if (node == 3) {
        offsetY += kNodeOffset;
      }
      targetYPos = tagPose.getY() - offsetY;
    }
    SmartDashboard.putNumber("Y Alignment Offset", offsetY);
    SmartDashboard.putNumber("Target Y Pos", targetYPos);
    xController.setSetpoint(14.4);
    yController.setSetpoint(targetYPos);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xDrive = xController.calculate(m_drivetrain.getFieldPosition().getX());
    var yDrive = yController.calculate(m_drivetrain.getFieldPosition().getY());
    var rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getDegrees());
    SmartDashboard.putString("XYtheta", xDrive+" "+yDrive+" "+rot);
    rot = MathUtil.clamp(rot, -0.4, 0.4);
    xDrive = MathUtil.clamp(xDrive, -0.4, 0.4);
    yDrive = MathUtil.clamp(yDrive, -0.4, 0.4);
    m_drivetrain.drive(xDrive, yDrive, rot, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!results.hasTargets()) {
      return true;
    }
    return false;
  }
}
