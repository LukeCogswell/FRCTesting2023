// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
  
  public static CommandBase ThreeGPBalanceNonCC(Drivetrain drivetrain, LEDs LEDs) {
    Constants.AUTO_EVENT_MAP.put("Cube", new InstantCommand(() -> LEDs.setLEDS(Color.kPurple)));
    Constants.AUTO_EVENT_MAP.put("Cone", new InstantCommand(() -> LEDs.setLEDS(Color.kYellow)));
    Constants.AUTO_EVENT_MAP.put("UpdateOdometry", new InstantCommand(() -> drivetrain.updateOdometryIfTag()));

    List<PathPlannerTrajectory> AutoPaths =
      PathPlanner.loadPathGroup(
        "3GPBalanceNonCC",
        Constants.MeasurementConstants.kMaxSpeedMetersPerSecond - 3,
        Constants.MeasurementConstants.kMaxAccelerationMetersPerSecondSquared);
    
        return Commands.sequence(
          // new InstantCommand(() -> drivetrain.setFieldPosition(new Pose2d(14.75, 5.07, new Rotation2d(Math.PI/2)))),
          new InstantCommand(() -> drivetrain.setFieldPosition(new Pose2d(14.75, 5.07, new Rotation2d()))),
          new PrintCommand(drivetrain.getFieldPosition().toString()),
          new WaitCommand(0.5).andThen(new InstantCommand(() -> LEDs.setLEDS(Color.kBlack))),
          new FollowPathWithEvents(
            drivetrain.getCommandForTrajectory(AutoPaths.get(0)),
            AutoPaths.get(0).getMarkers(), 
            Constants.AUTO_EVENT_MAP),
          new AlignWithNode(drivetrain, 2).andThen(new AimAtNode(drivetrain).withTimeout(0.3)),
          new WaitCommand(0.5).andThen(new InstantCommand(() -> LEDs.setLEDS(Color.kBlack))),
          new FollowPathWithEvents(
            drivetrain.getCommandForTrajectory(AutoPaths.get(1)),
            AutoPaths.get(1).getMarkers(),
            Constants.AUTO_EVENT_MAP),
          new AlignWithNode(drivetrain, 1).andThen(new AimAtNode(drivetrain).withTimeout(0.3)),
          new WaitCommand(0.5).andThen(new InstantCommand(() -> LEDs.setLEDS(Color.kBlack))),
          new FollowPathWithEvents(
            drivetrain.getCommandForTrajectory(AutoPaths.get(2)),
            AutoPaths.get(2).getMarkers(),
            Constants.AUTO_EVENT_MAP)
        );
  }


}
