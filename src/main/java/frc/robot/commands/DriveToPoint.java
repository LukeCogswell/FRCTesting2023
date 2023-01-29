// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

public class DriveToPoint extends CommandBase {
  private Drivetrain m_drivetrain;
  private double targetYPos;
  private double targetXPos;
  private Boolean useTagOd;
  private double targetRotation;
  private PIDController yController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController xController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController turnController = new PIDController(0.05, kSteerI, kSteerD);
  /** Creates a new DriveToPoint. */
  public DriveToPoint(Drivetrain drivetrain, Double x, Double y, Double rot, Boolean useTagOdometryUpdates) {
    targetYPos = y;
    targetXPos = x;
    targetRotation = rot;
    m_drivetrain = drivetrain;
    useTagOd = useTagOdometryUpdates;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.updateOdometryIfTag();
    turnController.enableContinuousInput(-180, 180);
    turnController.setTolerance(2);
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);

    xController.setSetpoint(targetXPos);
    yController.setSetpoint(targetYPos);
    turnController.setSetpoint(targetRotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() % 1 < 0.03 && useTagOd) {
      m_drivetrain.updateOdometryIfTag();
    }
    var xDrive = xController.calculate(m_drivetrain.getFieldPosition().getX());
    var yDrive = yController.calculate(m_drivetrain.getFieldPosition().getY());
    var rot = turnController.calculate(m_drivetrain.getFieldPosition().getRotation().getRadians());
    rot = MathUtil.clamp(rot, -1, 1);
    xDrive = MathUtil.clamp(xDrive, -1.7, 1.7);
    yDrive = MathUtil.clamp(yDrive, -1.7, 1.7);
    m_drivetrain.drive(xDrive, yDrive, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turnController.close();
    yController.close();
    xController.close();
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
