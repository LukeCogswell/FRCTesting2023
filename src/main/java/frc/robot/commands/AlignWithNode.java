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
  private Drivetrain m_drivetrain;
  private Integer node;
  private double targetYPos;
  private PIDController yController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController xController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController turnController = new PIDController(0.05, kSteerI, kSteerD);

  /**
   * Looks for the nearest node April tag and takes a given node number and drive toward it.
   * 
   * @param drivetrain - the current {@link Drivetrain}
   * @param whichNode - the desired node to travel to. 
   * Integer values of 1, 2 and 3 are applicable, with 1 and 2 being cone nodes.
   */
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
      return;
    } 

    m_drivetrain.updateOdometryIfTag();
    m_drivetrain.limelightToTagMode();
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(2);
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);

    if (node == 2) m_drivetrain.limelightToTagMode();
    else m_drivetrain.limelightToTapeMode();

    if (m_drivetrain.getTV() == 0) {
      this.cancel();
      return;
    }

    var tagID = m_drivetrain.getTID();
    var tagPose = Constants.AprilTagFieldLayouts.AprilTagList.get(tagID - 1).pose;
    var offsetY = getYOffset();

    if (tagID < 5) {
      xController.setSetpoint(14.5);
      turnController.setSetpoint(0);
      targetYPos = tagPose.getY() + offsetY;
    } else {
      xController.setSetpoint(2);
      turnController.setSetpoint(180);
      targetYPos = tagPose.getY() - offsetY;
    }
    yController.setSetpoint(targetYPos);
  }

  private double getYOffset() {
    switch (node) {
      case 1:
        return kNodeOffset;
      
      case 2:
        return 0.0;

      case 3:
        return -kNodeOffset;

      default:
        return 0.0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var position = m_drivetrain.getFieldPosition();
    var xDrive = xController.calculate(position.getX());
    var yDrive = yController.calculate(position.getY());
    var rot = turnController.calculate(position.getRotation().getDegrees());

    rot = MathUtil.clamp(rot, -1, 1);
    xDrive = MathUtil.clamp(xDrive, -1.4, 1.4);
    yDrive = MathUtil.clamp(yDrive, -1.4, 1.4);
    m_drivetrain.drive(xDrive, yDrive, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xController.atSetpoint() && yController.atSetpoint() && turnController.atSetpoint()) {
      return true;
    }
    return false;
  }
}
