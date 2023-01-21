package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonSRX topBarMotor = new TalonSRX(4);
  private final TalonSRX bottomBarMotor = new TalonSRX(5);
  //  private final WPI_TalonFX intakePivotMotor = new WPI_TalonFX(0);




  /** Creates a new ExampleSubsystem. */
  public Intake() {
   
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase IntakeCommand() {
    // Inline construction of command goes here.
    return startEnd(
        () -> {
          topBarMotor.set(ControlMode.PercentOutput, .5);
          bottomBarMotor.set(ControlMode.PercentOutput, -.5);
        },
        () -> {
            topBarMotor.set(ControlMode.PercentOutput, 0);
            bottomBarMotor.set(ControlMode.PercentOutput, 0);
        }
        );
  }
  public CommandBase OuttakeCommand() {

  return startEnd(
    () -> {
      topBarMotor.set(ControlMode.PercentOutput, -.5);
      bottomBarMotor.set(ControlMode.PercentOutput, .5);
    },
    () -> {
        topBarMotor.set(ControlMode.PercentOutput, 0);
        bottomBarMotor.set(ControlMode.PercentOutput, 0);
    }
    );
}
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}









//public class Intake {
    //need one talon for the inner bar
    //need one talon for outer bar
    //need one for pivoting? later
    //need button to make both spin ot pull in
    //need to hold it?
    //need button to spit it out
    //need to give them can ids or whatever to connect them
//}