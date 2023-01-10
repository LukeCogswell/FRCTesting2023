// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.BalancingConstants.*;

public class BalanceRobotOnChargingStation extends CommandBase {
  private Drivetrain m_swerveDrive;
  private DoubleSupplier adjustmentStrength;
  /** Creates a new BalanceRobotOnChargingStation. */
  public BalanceRobotOnChargingStation(Drivetrain swerveDrive, DoubleSupplier tiltAdjustmentStrength) {
    m_swerveDrive = swerveDrive;
    adjustmentStrength = tiltAdjustmentStrength;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var strength = adjustmentStrength.getAsDouble() * kTriggerMultiplier;
    var pitch = m_swerveDrive.getNavxPitch();
    var roll = -m_swerveDrive.getNavxRoll();
    SmartDashboard.putNumber("Pitch", pitch);
    SmartDashboard.putNumber("Roll", roll);
    pitch = -kAngleTolerance > pitch || pitch > kAngleTolerance ? pitch * strength: 0;
    roll = -kAngleTolerance > roll || roll > kAngleTolerance ? roll * strength: 0;
    pitch = MathUtil.clamp(pitch, -kSpeedLimit, kSpeedLimit);
    roll = MathUtil.clamp(roll, -kSpeedLimit, kSpeedLimit);
    m_swerveDrive.drive(
      pitch,
      // 0,
      roll,
      0,
      false);
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
