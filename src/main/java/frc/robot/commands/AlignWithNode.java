// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.MeasurementConstants.*;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

public class AlignWithNode extends CommandBase {
  /** Creates a new AlignWithNode. */
  private Drivetrain m_drivetrain;
  private Integer node;
  private double targetYPos;
  private PIDController yController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController xController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController turnController = new PIDController(0.05, kSteerI, kSteerD);


  public AlignWithNode(Drivetrain drivetrain, Integer whichNode) {
    node = whichNode;
    m_drivetrain = drivetrain;

    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_drivetrain.botPoseSub.get().length != 6) {
      System.out.println("STOPPINGGGGGG");
      this.cancel();
    } else {
    m_drivetrain.updateOdometryIfTag();
    m_drivetrain.limelightToTagMode();
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(2);
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    if (node == 2) {
      m_drivetrain.limelightToTagMode();
    } else {
      m_drivetrain.limelightToTapeMode();
    }
    if (m_drivetrain.getTV() == 1) {
    var tagID = m_drivetrain.getTID();
    var tagPose = Constants.AprilTagFieldLayouts.AprilTagList.get(tagID - 1).pose;
    var offsetY = 0.0;
    if (tagID < 5) {
      xController.setSetpoint(14.5);
      turnController.setSetpoint(0);
      if (node == 1 ) {
        offsetY -= kNodeOffset; 
      } else if (node == 3) {
        offsetY += kNodeOffset;
      }
      targetYPos = tagPose.getY() + offsetY;
    } else {
      xController.setSetpoint(2);
      turnController.setSetpoint(180);
      if (node == 3 ) {
        offsetY -= kNodeOffset; 
      } else if (node == 1) {
        offsetY += kNodeOffset;
      }
      targetYPos = tagPose.getY() - offsetY;
    }
    yController.setSetpoint(targetYPos);
    } else {
      this.cancel();
    }}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (Timer.getFPGATimestamp() % 1 < 0.03){
    //   m_drivetrain.updateOdometryIfTag();

    // }
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
    if (interrupted) {
      m_drivetrain.limelightToTagMode();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xController.atSetpoint() && yController.atSetpoint() && turnController.atSetpoint()) {
      return true;
    }
    return false;
  }
}
