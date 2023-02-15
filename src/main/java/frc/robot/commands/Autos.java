// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Intake.IntakeMode;
import frc.robot.subsystems.Intake.PivotPosition;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public final class Autos {
  private static final PathConstraints kDefaultConstraints = new PathConstraints(4, 4);
  private static final PathConstraints kSlowConstraints = new PathConstraints(1, 4);
  private static final PathConstraints kFastConstraints = new PathConstraints(3.81, 8);

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

  public static Command rightSide(Drive drive, Intake intake, Arm arm) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Intake", new ParallelCommandGroup(intake.autoCommand(PivotPosition.DOWN, IntakeMode.IN), arm.armCommand(ArmPosition.HAND_OFF)));
    eventMap.put("HandOff", intake.autoCommand(PivotPosition.UP, IntakeMode.STOP));
    eventMap.put("Middle", arm.armCommand(ArmPosition.MEDIUM, true));
    eventMap.put("Score", arm.scoreCommand());
    eventMap.put("High", arm.armCommand(ArmPosition.HIGH));

    return setupAuto("Right Side", eventMap, drive,
      kDefaultConstraints, //start to bump
      kSlowConstraints, //cross bump 1
      kDefaultConstraints, //to cone and back
      kSlowConstraints, //cros bump 2
      kDefaultConstraints  //back to start/end
    );
  }

  public static Command leftSide(Drive drive, Intake intake, Arm arm) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Intake", new ParallelCommandGroup(intake.autoCommand(PivotPosition.DOWN, IntakeMode.IN), arm.armCommand(ArmPosition.HAND_OFF)));
    eventMap.put("HandOff", intake.autoCommand(PivotPosition.UP, IntakeMode.STOP));
    eventMap.put("Middle", arm.armCommand(ArmPosition.MEDIUM, true));
    eventMap.put("Score", arm.scoreCommand());
    eventMap.put("High", arm.armCommand(ArmPosition.HIGH));

    return setupAuto("Right Side", eventMap, drive,
      kFastConstraints
    );
  }
  private static Command setupAuto(String pathName, HashMap<String, Command> eventMap, Drive drive) {
    return setupAuto(pathName, eventMap, drive, kDefaultConstraints);
  }
  
  private static Command setupAuto(String pathName, HashMap<String, Command> eventMap, Drive drive, PathConstraints constraint, PathConstraints... constraints) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, 
      constraint, constraints
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
