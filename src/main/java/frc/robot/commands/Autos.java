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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public final class Autos {
  private static final PathConstraints kDefaultConstraints = new PathConstraints(4.33, 3.6);
  private static final PathConstraints kSlowAccConstraints = new PathConstraints(4.33, 2.5);
  private static final PathConstraints kMedium1Constraints = new PathConstraints(2, 3);
  private static final PathConstraints kBump1Constraints = new PathConstraints(2.8, 3);
  private static final PathConstraints kMedium2Constraints = new PathConstraints(4.33, 3.3);
  private static final PathConstraints kSlowConstraints = new PathConstraints(1, 4);
  private static final PathConstraints kBalanceConstraints = new PathConstraints(3, 1);
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

  public static Command bump_Balance(Drive drive, Intake intake, Arm arm, RobotState robotState) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("ConeMode", new SetConeModeCommand(robotState, true));
    eventMap.put("CubeMode", new SetCubeModeCommand(robotState, true));
    eventMap.put("Intake", new SetCubeModeCommand(robotState, true).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, ArmPosition.HANDOFF_CUBE))));
    eventMap.put("Middle", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.MIDDLE, true));
    eventMap.put("ScoreMiddle", new ArmScoreCommand(arm));
    eventMap.put("High", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true));
    eventMap.put("HighWait", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true, .9));
    eventMap.put("ScoreHigh", new ArmScoreCommand(arm));
    eventMap.put("FirstDrop", new ArmMoveCommand(arm, -.5, ArmPosition.AUTO_DROP));
    eventMap.put("IntakeUp", intake.setPivot(PivotPosition.UP));
    eventMap.put("Crab", new DriveCrabCommand(drive));
    
    return setupAuto("Bump - Balance", eventMap, drive,
      kDefaultConstraints, //start to bump
      kSlowConstraints, //cross bump 1
      kDefaultConstraints, //to cone and back
      kDefaultConstraints, //to cone and back
      kSlowConstraints, //cros bump 2
      kDefaultConstraints, //
      kDefaultConstraints,  //back to start/end
      kSlowConstraints, //cros bump 2
      kDefaultConstraints //cros bump 2
    );
  }


  public static Command bump(Drive drive, Intake intake, Arm arm, RobotState robotState) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("ConeMode", new SetConeModeCommand(robotState, true));
    eventMap.put("CubeMode", new SetCubeModeCommand(robotState, true));
    eventMap.put("Intake", new SetCubeModeCommand(robotState, true).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, ArmPosition.HANDOFF_CUBE))));
    eventMap.put("Middle", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.MIDDLE, true));
    eventMap.put("ScoreMiddle", new ArmScoreCommand(arm));
    eventMap.put("High", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true));
    eventMap.put("HighWait", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true, .9));
    eventMap.put("ScoreHigh", new ArmScoreCommand(arm));
    eventMap.put("FirstDrop", new ArmMoveCommand(arm, -.5, ArmPosition.AUTO_DROP));
    eventMap.put("IntakeUp", intake.setPivot(PivotPosition.UP));
    eventMap.put("Crab", new DriveCrabCommand(drive));

    return setupAuto("Bump", eventMap, drive,
      kDefaultConstraints, //start to bump
      kSlowConstraints, //cross bump 1
      kDefaultConstraints, //to cone and back
      kDefaultConstraints, //to cone and back
      kSlowConstraints, //cros bump 2
      kDefaultConstraints, //
      kDefaultConstraints,  //back to start/end
      kSlowConstraints, //cros bump 2
      kDefaultConstraints //cros bump 2
    );
  }

  public static Command bump_3Piece(Drive drive, Intake intake, Arm arm, RobotState robotState) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("IntakeCube", new SetCubeModeCommand(robotState, true).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, ArmPosition.HANDOFF_CUBE))));
    eventMap.put("IntakeCone", new SetConeModeCommand(robotState, true).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, CommandMode.WAIT, ArmPosition.HANDOFF_CONE1))));
    eventMap.put("High", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true));
    eventMap.put("Middle", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.MIDDLE, true));
    eventMap.put("StartScore", arm.handCommand(false, -.5));
    eventMap.put("HighWait", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true, .9));
    eventMap.put("ScoreHigh", new DriveVisionCommand(drive, true).alongWith(new ArmScoreCommand(arm)));
    eventMap.put("IntakeUp", intake.setPivot(PivotPosition.UP));
    eventMap.put("Crab", new DriveCrabCommand(drive));
    eventMap.put("Handoff", new ArmMoveAfterIntakeCommand(arm, intake));


    return setupAuto("Bump - 3 Piece", eventMap, drive,
      kBump1Constraints //start to bump
      //kSlowConstraints, //cross bump 1
      //kDefaultConstraints, //to cone and back
      //kDefaultConstraints, //to cone and back
      //kSlowConstraints, //cros bump 2
      //kDefaultConstraints, //
      //kDefaultConstraints,  //back to start/end
      //kSlowConstraints, //cros bump 2
      //kDefaultConstraints, //cros bump 2
      //kDefaultConstraints,//grab 2nd
      //kSlowConstraints,//cross bump 3?
      //kDefaultConstraints,//crossed bump
      //kDefaultConstraints//done
    );
  }

  public static Command bump_2_5_Balance(Drive drive, Intake intake, Arm arm, RobotState robotState) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("IntakeCube", new SetCubeModeCommand(robotState, true).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, ArmPosition.HANDOFF_CUBE))));
    eventMap.put("IntakeCone", new SetConeModeCommand(robotState, true).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, CommandMode.WAIT, ArmPosition.HANDOFF_CONE1))));
    eventMap.put("High", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true));
    eventMap.put("HighWait", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true, .9));
    eventMap.put("ScoreHigh", new DriveVisionCommand(drive, true).alongWith(new ArmScoreCommand(arm)));
    eventMap.put("IntakeUp", intake.setPivot(PivotPosition.UP));
    eventMap.put("Crab", new DriveCrabCommand(drive));
    eventMap.put("Handoff", new ArmMoveAfterIntakeCommand(arm, intake));
    eventMap.put("Balance", new DriveBalanceCommand(drive, true));
    
    return setupAuto("Bump - 2_5 Balance", eventMap, drive,
      kBump1Constraints,
      kBump1Constraints,
      kBump1Constraints,
      kBump1Constraints,
      kBump1Constraints
      //start to bump
      //kSlowConstraints, //cross bump 1
      //kDefaultConstraints, //to cone and back
      //kDefaultConstraints, //to cone and back
      //kSlowConstraints, //cros bump 2
      //kDefaultConstraints, //
      //kDefaultConstraints,  //back to start/end
      //kSlowConstraints, //cros bump 2
      //kDefaultConstraints, //cros bump 2
      //kDefaultConstraints,//grab 2nd
      //kSlowConstraints,//cross bump 3?
      //kDefaultConstraints,//crossed bump
      //kDefaultConstraints//done
    );
  }

  public static Command balance(Drive drive, Intake intake, Arm arm, RobotState robotState) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Balance", new DriveBalanceCommand(drive, true));

    return setupAuto("Balance", eventMap, drive,
      kBump1Constraints
    );
  }

  public static Command gap_2_5PiecesBalance(Drive drive, Intake intake, Arm arm, RobotState robotState) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("IntakeCube", new SetCubeModeCommand(robotState, true).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, ArmPosition.HANDOFF_CUBE))));
    eventMap.put("IntakeCone", new SetConeModeCommand(robotState, true).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, CommandMode.WAIT, ArmPosition.HANDOFF_CONE1))));
    eventMap.put("HandoffHigh", new ArmMoveAfterIntakeCommand(arm, intake));
    eventMap.put("High", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, false));
    eventMap.put("HighWait", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true, .9));
    eventMap.put("ScoreHigh", new ArmScoreCommand(arm));
    eventMap.put("IntakeUp", intake.setPivot(PivotPosition.UP));
    eventMap.put("Balance", new DriveBalanceCommand(drive, true));
    
    return setupAuto("Gap - 2_5 Balance", eventMap, drive,
      kDefaultConstraints
    );
  }

  public static Command gap_3Pieces(Drive drive, Intake intake, Arm arm, RobotState robotState) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("HighWait", new DriveVisionCommand(drive, true).alongWith(new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true, .9).andThen(new ArmScoreCommand(arm, true))));
    eventMap.put("IntakeCube", new SetCubeModeCommand(robotState, true).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, ArmPosition.HANDOFF_CUBE))));
    eventMap.put("Middle", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.MIDDLE, false));
    eventMap.put("Score", new ArmScoreCommand(arm, true));
    eventMap.put("High", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, false));
    //eventMap.put("ScoreHigh", new ArmScoreCommand(arm));
    eventMap.put("IntakeUp", intake.setPivot(PivotPosition.UP));
    
    return setupAuto("Gap - 3 Piece", eventMap, drive,
    kSlowAccConstraints,
    kDefaultConstraints
    );
  }

  public static Command gap_Link(Drive drive, Intake intake, Arm arm, RobotState robotState) {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("IntakeCube", new SetCubeModeCommand(robotState, true).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, ArmPosition.HANDOFF_CUBE))));
    eventMap.put("IntakeCone", new SetConeModeCommand(robotState, true).andThen(intake.intakeCommand(true).alongWith(new ArmMoveCommand(arm, .9, CommandMode.WAIT, ArmPosition.HANDOFF_CONE1))));
    eventMap.put("HandoffHigh", new ArmMoveAfterIntakeCommand(arm, intake, ArmPosition.HIGH_CONE));
    eventMap.put("High", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, false));
    eventMap.put("HighWait", new ArmMoveToScoreCommand(arm, robotState, ScoreMode.HIGH, true, .9));
    eventMap.put("ScoreHigh", new ArmScoreCommand(arm));
    eventMap.put("IntakeUp", intake.setPivot(PivotPosition.UP));
    
    String pathName = "Gap - Link";
    if(DriverStation.getAlliance() == Alliance.Red) {
      pathName += " Red";
    }

    return setupAuto(pathName, eventMap, drive,
      kDefaultConstraints, kMedium2Constraints, kDefaultConstraints, kMedium1Constraints
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
