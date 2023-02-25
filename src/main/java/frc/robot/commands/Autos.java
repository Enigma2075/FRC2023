// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.ArmMoveCommand.CommandMode;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotState;
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
  private static final PathConstraints kDefaultConstraints = new PathConstraints(4, 3);
  private static final PathConstraints kMediumConstraints = new PathConstraints(1.5, 3);
  private static final PathConstraints kSlowConstraints = new PathConstraints(1, 4);
  private static final PathConstraints kFastConstraints = new PathConstraints(4, 3);

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
    eventMap.put("Intake", new ParallelCommandGroup(intake.autoCommand(PivotPosition.DOWN, IntakeMode.CONE_IN), new ArmMoveCommand(arm, ArmPosition.HAND_OFF)));
    eventMap.put("HandOff", intake.autoCommand(PivotPosition.UP, IntakeMode.STOP));
    eventMap.put("Middle", new ArmMoveCommand(arm, CommandMode.WAIT, ArmPosition.MEDIUM));
    eventMap.put("Score", arm.scoreCommand());
    eventMap.put("High", new ArmMoveCommand(arm, CommandMode.WAIT, ArmPosition.HIGH1, ArmPosition.HIGH2));

    return setupAuto("Right Side", eventMap, drive,
      kDefaultConstraints, //start to bump
      kSlowConstraints, //cross bump 1
      kDefaultConstraints, //to cone and back
      kSlowConstraints, //cros bump 2
      kDefaultConstraints  //back to start/end
    );
  }

  public static Command leftSide(Drive drive, Intake intake, Arm arm, RobotState robotstate) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Intake", new SetConeModeCommand(robotstate).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, CommandMode.WAIT, ArmPosition.HANDOFF2_CONE1))));
    eventMap.put("Handoff", new ArmMoveAfterIntakeCommand(arm, intake));
    eventMap.put("Middle", new ArmMoveCommand(arm, .9, CommandMode.WAIT, ArmPosition.MEDIUM));
    eventMap.put("ScoreMiddle", new ArmMoveCommand(arm, CommandMode.WAIT, ArmPosition.SCORE_OFFSET).andThen(arm.scoreCommand()));
    eventMap.put("ScoreHigh", arm.scoreCommand());
    eventMap.put("High", new ArmMoveCommand(arm, CommandMode.WAIT, ArmPosition.HIGH2));

    return setupAuto("Left Side Test2", eventMap, drive,
      kDefaultConstraints, kMediumConstraints, kDefaultConstraints, kMediumConstraints
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
      new PIDConstants(.9, 0, 0), 
      drive::setWpiModuleStates, 
      eventMap,
      drive);

      return autoBuilder.fullAuto(pathGroup);
  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
