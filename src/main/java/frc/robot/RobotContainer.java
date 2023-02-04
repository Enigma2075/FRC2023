// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.other.Subsystem;
import frc.robot.Constants.DriverStation;
import frc.robot.commands.ArmButtonCommand;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveDefaultCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cancoders;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.Intake;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    mDrive.setDefaultCommand(new DriveDefaultCommand(mDrive, mDriverController::getLeftY, mDriverController::getLeftX, mDriverController::getRightX, mDriverController.x()));

    PathPlannerServer.startServer(5811);
    
    mIntake = new Intake();
    mArm = new Arm();
    mArm.setDefaultCommand(new ArmManualCommand(mArm, mOperatorController::getLeftY, mOperatorController::getRightY));

    setSubsystems(mDrive, mArm);
    
    mAutoChooser = new SendableChooser<>();
    mAutoChooser.setDefaultOption("Straight", Autos.straightTest(mDrive));
    mAutoChooser.addOption("Spline", Autos.splineTest(mDrive));

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    //mDriverController.a().whileTrue(mIntake.IntakeCommand());
    //mDriverController.b().whileTrue(mIntake.OuttakeCommand());
    
    //mOperatorController.a().whileTrue(new ArmButtonCommand(mArm, mArm.calcShoulderArbFF())); //angle 0.16 works?
    //mOperatorController.b().whileTrue(new ArmButtonCommand(mArm, mArm.calcElbowArbFF()));
    mOperatorController.b().whileTrue(new ArmButtonCommand(mArm, -30, -115));
    mOperatorController.x().whileTrue(new ArmButtonCommand(mArm, 0, 0));
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
