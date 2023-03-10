// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectoryWithEvents extends CommandBase {

  Drivetrain m_swerveDriveController;

  String m_trajectoryName;

  HashMap<String, Command> m_events;
  /** Creates a new FollowTrajectoryWithEvents. */
  public FollowTrajectoryWithEvents(Drivetrain swerveDriveController, String trajectoryName) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerveDriveController = swerveDriveController;
    m_trajectoryName = trajectoryName;
    addRequirements(m_swerveDriveController);
    m_events = Constants.AUTO_EVENT_MAP;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerTrajectory path = PathPlanner.loadPath(m_trajectoryName, new PathConstraints(4, 3));

    FollowPathWithEvents followPath = new FollowPathWithEvents(
      m_swerveDriveController.getCommandForTrajectory(path), 
      path.getMarkers(), 
      m_events);
    
    followPath.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
