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
  public static Command straightTest(Drive drive) {
    HashMap<String, Command> eventMap = new HashMap<>();

    return setupAuto("Straight Test", eventMap, drive);
  }
  
  public static Command splineTest(Drive drive) {
    HashMap<String, Command> eventMap = new HashMap<>();

    return setupAuto("Spline Test", eventMap, drive);
  }
  
  public static Command strafeTest(Drive drive) {
    HashMap<String, Command> eventMap = new HashMap<>();

    return setupAuto("Strafe Test", eventMap, drive);
  }

  public static Command rightSide(Drive drive) {
    HashMap<String, Command> eventMap = new HashMap<>();

    return setupAuto("Right Side", eventMap, drive);
  }

  private static Command setupAuto(String pathName, HashMap<String, Command> eventMap, Drive drive) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, 
    new PathConstraints(4, 4), //start to bump
    new PathConstraints(1, 4), //cross bump 1
    new PathConstraints(4, 4), //to cone and back
    new PathConstraints(1, 4), //cros bump 2
    new PathConstraints(4, 4)  //back to start/end
    );
    
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drive::getWpiPose, 
      drive::resetWpiPose, 
      frc.robot.Constants.Drive.kKinematics.asWpiSwerveDriveKinematics(), 
      new PIDConstants(5, 0, 0), 
      new PIDConstants(.5, 0, 0), 
      drive::setWpiModuleStates, 
      eventMap,
      drive);

      return autoBuilder.fullAuto(pathGroup);
  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
