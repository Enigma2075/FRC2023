// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.other.Subsystem;
import frc.robot.Constants.DriverStation;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.ArmMoveAfterIntakeCommand;
import frc.robot.commands.ArmMoveCommand;
import frc.robot.commands.ArmScoreCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveDefaultCommand;
import frc.robot.commands.SetConeModeCommand;
import frc.robot.commands.SetCubeModeCommand;
import frc.robot.commands.ArmMoveCommand.CommandMode;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cancoders;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.Intake.PivotPosition;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotState;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.Arm.ScoreMode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private List<Subsystem> mAllSubsystems = new ArrayList<>();
    
  private final Cancoders mCancoders;
  private final Drive mDrive;
  private final Intake mIntake;
  private final Arm mArm;
  private final RobotState mRobotState;
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController mDriverController =
      new CommandXboxController(DriverStation.kDriverControllerPort);
  private final CommandXboxController mOperatorController = 
      new CommandXboxController(DriverStation.kOperatorControllerPort);

  private SendableChooser<Command> mAutoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Dirty swerve init hack step 1: WaitForNumBannerSensorsAction for cancoders to init
    mCancoders = Cancoders.getInstance();
    double startInitTs = Timer.getFPGATimestamp();
    System.out.println("* Starting to init cancoders at ts " +  startInitTs);
    while (Timer.getFPGATimestamp() - startInitTs < Constants.Drive.kCancoderBootAllowanceSeconds && !mCancoders.allHaveBeenInitialized()) {
        Timer.delay(0.1);
    }
    System.out.println("* Cancoders all inited: Took " + (Timer.getFPGATimestamp() - startInitTs) + " seconds");

    // Dirty swerve init hack step 2: Build all the rest of the subsystems
    mDrive = Drive.getInstance();
    mDrive.setDefaultCommand(new DriveDefaultCommand(mDrive, mDriverController::getLeftY, mDriverController::getLeftX, mDriverController::getRightX, mDriverController::getRightY, mDriverController.x(), mDriverController.povRight()));

    PathPlannerServer.startServer(5811);
    
    mRobotState = new RobotState();
    mIntake = new Intake(mRobotState);
    mArm = new Arm(mRobotState);

    mArm.setIntake(mIntake);
    mIntake.setArm(mArm);
    
    mArm.setDefaultCommand(new ArmDefaultCommand(mArm));

    //mIntake.setDefaultCommand(mIntake.defaultCommand(mDriverController::getRightTriggerAxis, mDriverController::getLeftTriggerAxis));
    //mIntake.setDefaultCommand(mIntake.testPivotCommand(mOperatorController::getRightX));
    
    //mArm.setDefaultCommand(new ArmManualCommand(mArm, mOperatorController::getLeftY, mOperatorController::getRightY));

    setSubsystems(mDrive, mIntake, mArm, mRobotState);
    
    mAutoChooser = new SendableChooser<>();
    mAutoChooser.setDefaultOption("Left - 3 Pieces Balance", Autos.leftSide_3PiecesBalance(mDrive, mIntake, mArm, mRobotState));
    mAutoChooser.addOption("Left - 4 Pieces", Autos.leftSide_4Pieces(mDrive, mIntake, mArm, mRobotState));
    mAutoChooser.addOption("Left - 3 Cone", Autos.leftSide_3Cone(mDrive, mIntake, mArm, mRobotState));
    mAutoChooser.addOption("Right", Autos.rightSide(mDrive, mIntake, mArm, mRobotState));
    mAutoChooser.addOption("Straight", Autos.straightTest(mDrive));
    mAutoChooser.addOption("Spline", Autos.splineTest(mDrive));
    mAutoChooser.addOption("Strafe", Autos.strafeTest(mDrive));
    
    SmartDashboard.putData(mAutoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // CONE MODE
    mDriverController.b().or(mOperatorController.rightBumper()).whileTrue(new SetConeModeCommand(mRobotState));
    // CUBE MODE
    mDriverController.a().or(mOperatorController.leftBumper()).whileTrue(new SetCubeModeCommand(mRobotState));

    // FLOOR INTAKE
    new Trigger(mDriverController.rightTrigger(.7)).onTrue(new ConditionalCommand(mIntakeCubeStart(), mIntakeConeStart(), mRobotState::isCubeMode).alongWith(mIntake.intakeCommand())).debounce(.125).onFalse(new ConditionalCommand(mIntakeCubeEnd(), mIntakeConeEnd(), mRobotState::isCubeMode)).debounce(.125);
    // OUTTAKE
    new Trigger(mDriverController.leftTrigger(.7)).whileTrue(mIntake.outtakeCommand());
    
    // SCORE
    mDriverController.leftBumper().onTrue(new ArmScoreCommand(mArm).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)).debounce(.5);

    // SHELF
    new Trigger(mOperatorController.rightTrigger(.7)).onTrue(new ArmMoveCommand(mArm, .9, ArmPosition.SHELF)).debounce(.5).onFalse(new ConditionalCommand(new ArmMoveCommand(mArm, ArmPosition.HOLD), new ArmMoveCommand(mArm, ArmPosition.DEFAULT), mArm::handHasGamePeice)).debounce(.5);
    // FEEDER
    new Trigger(mOperatorController.leftTrigger(.7)).onTrue(new ArmMoveCommand(mArm, .9, ArmPosition.HANDOFF_CONE1).alongWith(mIntake.intakeFeederCommand())).debounce(.5).onFalse(mIntakeConeEnd()).debounce(.5);

    // MIDDLE
    mOperatorController.b().onTrue(mArm.moveToScore(ScoreMode.MIDDLE));
    // HIGH
    mOperatorController.y().onTrue(mArm.moveToScore(ScoreMode.HIGH));
    // HAND
    mOperatorController.a().whileTrue(mArm.handCommand());
    
    // DEFAULT
    mOperatorController.x().whileTrue(new ArmMoveCommand(mArm, ArmPosition.DEFAULT_SHOULDER, ArmPosition.DEFAULT_ELBOW));
  }

  private final Command mIntakeConeStart() {
    return new ArmMoveCommand(mArm, .9, CommandMode.DONT_END, ArmPosition.HANDOFF_CONE1);
  }

  private final Command mIntakeConeEnd() {
    return new ArmMoveAfterIntakeCommand(mArm, mIntake);
  }

  private final Command mIntakeCubeStart() {
    return new ArmMoveCommand(mArm, .7, ArmPosition.HANDOFF_CUBE);
  }

  private final Command mIntakeCubeEnd() {
    return mIntake.setPivot(PivotPosition.UP).alongWith(new ArmMoveCommand(mArm, ArmPosition.DEFAULT));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return mAutoChooser.getSelected();
  }

  public void setSubsystems(Subsystem... allSubsystems) {
    mAllSubsystems = Arrays.asList(allSubsystems);
  }

  public void readPeriodicInputs() {
    mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
  }

  public void writePeriodicOutputs() {
    mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);
  }

  public void stop() {
    mAllSubsystems.forEach(Subsystem::stop);
  }

  public void autonomousInit() {
    mDrive.setControlState(DriveControlState.PATH_FOLLOWING);
  }

  public void teleopInit() {
  }

  public void outputTelemetry() {
    mAllSubsystems.forEach(Subsystem::outputTelemetry);
  }
}
