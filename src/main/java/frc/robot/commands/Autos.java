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
import frc.robot.subsystems.Arm.ScoreMode;
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

public final class Autos {
  private static final PathConstraints kDefaultConstraints = new PathConstraints(4.2, 3.2);
  private static final PathConstraints kMedium1Constraints = new PathConstraints(1.5, 3);
  private static final PathConstraints kMedium2Constraints = new PathConstraints(1.5, 3);
  private static final PathConstraints kSlowConstraints = new PathConstraints(1, 4);
  private static final PathConstraints kFastConstraints = new PathConstraints(4, 3);

  public static Command straightTest(Drive drive) {
    HashMap<String, Command> eventMap = new HashMap<>();

    return setupAuto("Straight Test", eventMap, drive
    );
  }
  
  public static Command splineTest(Drive drive) {
    HashMap<String, Command> eventMap = new HashMap<>();

    return setupAuto("Spline Test", eventMap, drive);
  }
  
  public static Command strafeTest(Drive drive) {
    HashMap<String, Command> eventMap = new HashMap<>();

    return setupAuto("Strafe Test", eventMap, drive);
  }

  public static Command rightSide(Drive drive, Intake intake, Arm arm, RobotState robotState) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("ConeMode", new SetConeModeCommand(robotState));
    eventMap.put("CubeMode", new SetCubeModeCommand(robotState));
    eventMap.put("Intake", new SetCubeModeCommand(robotState).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, ArmPosition.HANDOFF_CUBE))));
    eventMap.put("Middle", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.MIDDLE, true));
    eventMap.put("ScoreMiddle", new ArmScoreCommand(arm));
    eventMap.put("High", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true));
    eventMap.put("ScoreHigh", new ArmScoreCommand(arm));
    eventMap.put("FirstDrop", new ArmMoveCommand(arm, -.5, ArmPosition.AUTO_DROP));
    eventMap.put("IntakeUp", intake.setPivot(PivotPosition.UP));
    eventMap.put("Crab", new DriveCrabCommand(drive));

    return setupAuto("Right Side", eventMap, drive,
      kDefaultConstraints, //start to bump
      kSlowConstraints, //cross bump 1
      kDefaultConstraints, //to cone and back
      kDefaultConstraints, //to cone and back
      kSlowConstraints, //cros bump 2
      kDefaultConstraints, //
      kDefaultConstraints,  //back to start/end
      kSlowConstraints //cros bump 2
    );
  }

  public static Command leftSide_3PiecesBalance(Drive drive, Intake intake, Arm arm, RobotState robotState) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("ConeMode", new SetConeModeCommand(robotState));
    eventMap.put("CubeMode", new SetCubeModeCommand(robotState));
    eventMap.put("Intake", new SetCubeModeCommand(robotState).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, ArmPosition.HANDOFF_CUBE))));
    eventMap.put("Middle", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.MIDDLE, true));
    eventMap.put("ScoreMiddle", new ArmScoreCommand(arm));
    eventMap.put("High", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true));
    eventMap.put("ScoreHigh", new ArmScoreCommand(arm));
    eventMap.put("FirstDrop", new ArmMoveCommand(arm, -.5, ArmPosition.AUTO_DROP));
    eventMap.put("IntakeUp", intake.setPivot(PivotPosition.UP));
    eventMap.put("Crab", new DriveCrabCommand(drive));
    
    return setupAuto("Left Side - 3 Piece Balance", eventMap, drive,
      kDefaultConstraints, kDefaultConstraints, kDefaultConstraints, kDefaultConstraints, kDefaultConstraints, kSlowConstraints
    );
  }

  public static Command leftSide_4Pieces(Drive drive, Intake intake, Arm arm, RobotState robotState) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("ConeMode", new SetConeModeCommand(robotState));
    eventMap.put("CubeMode", new SetCubeModeCommand(robotState));
    eventMap.put("Intake", new SetCubeModeCommand(robotState).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, ArmPosition.HANDOFF_CUBE))));
    eventMap.put("Middle", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.MIDDLE, true));
    eventMap.put("ScoreMiddle", new ArmScoreCommand(arm));
    eventMap.put("High", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true));
    eventMap.put("ScoreHigh", new ArmScoreCommand(arm));
    eventMap.put("FirstDrop", new ArmMoveCommand(arm, -.5, ArmPosition.AUTO_DROP));
    eventMap.put("IntakeUp", intake.setPivot(PivotPosition.UP));
    //eventMap.put("Crab", new DriveCrabCommand(drive));
    
    return setupAuto("Left Side - 4 Piece", eventMap, drive,
      kDefaultConstraints
    );
  }

  public static Command leftSide_3Cone(Drive drive, Intake intake, Arm arm, RobotState robotstate) {
    HashMap<String, Command> eventMap = new HashMap<>();
    //eventMap.put("Intake", new SetConeModeCommand(robotstate).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, CommandMode.WAIT, ArmPosition.HANDOFF_CONE1))));
    //eventMap.put("Handoff", new ArmMoveAfterIntakeCommand(arm, intake, ArmPosition.HIGH_CONE));
    //eventMap.put("Middle", new ArmMoveCommand(arm, .9, CommandMode.WAIT, ArmPosition.MEDIUM_AUTO_START, ArmPosition.MEDIUM_CONE));
    //eventMap.put("ScoreMiddle", new ArmMoveCommand(arm, CommandMode.WAIT, ArmPosition.SCORE_OFFSET_CONE_MID).andThen(new ArmScoreCommand(arm)));
    //eventMap.put("ScoreHigh", new ArmScoreCommand(arm));
    
    return setupAuto("Left Side - 3 Cone", eventMap, drive,
      kDefaultConstraints, kMedium1Constraints, kDefaultConstraints, kMedium2Constraints
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
      new PIDConstants(8, 0, 0), 
      new PIDConstants(2, 0, 0), 
      drive::setWpiModuleStates, 
      eventMap,
      true,
      drive);

      return autoBuilder.fullAuto(pathGroup);
  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
