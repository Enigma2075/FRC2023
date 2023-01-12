// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drive;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command straightTest(Drive drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Straight Test", new PathConstraints(4, 4));
    
    HashMap<String, Command> eventMap = new HashMap<>();

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drive::getWpiPose, 
      drive::resetWpiPose, 
      frc.robot.Constants.Drive.kKinematics.asWpiSwerveDriveKinematics(), 
      new PIDConstants(5, 0, 0), 
      new PIDConstants(.5, 0, 0), 
      drive::setModuleStates, 
      eventMap,
      drive);

      return autoBuilder.fullAuto(pathGroup);
  }
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
