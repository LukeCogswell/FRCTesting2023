// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.SwerveModuleConstants.PID.*;

public class AimAtNode extends CommandBase {
  private Drivetrain m_drivetrain;
  private PIDController driveController = new PIDController(kDriveP, kDriveI, kDriveD);
  private PIDController rotController = new PIDController(0.05, 0, 0);
  /** Creates a new AimAtNode. */

  public AimAtNode(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_drivetrain.getTV() == 0) {
      this.cancel();
    }
    rotController.setSetpoint(0);
    rotController.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var rot = rotController.calculate(m_drivetrain.getTX());
    rot = MathUtil.clamp(rot, -0.8, 0.8);
    
    var yDrive = driveController.calculate(m_drivetrain.getTX());
    yDrive = MathUtil.clamp(yDrive, -0.2, 0.2);
    
    // m_drivetrain.drive(0, 0, rot);
    m_drivetrain.drive(0, yDrive, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.limelightToTagMode();
    rotController.close();
    driveController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (driveController.atSetpoint()) {
      return true;
    }
    return false;
  }
}
