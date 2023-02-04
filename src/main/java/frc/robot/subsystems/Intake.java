package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {//swhere you make it
  private final TalonSRX intakeBarMotor;
  private final TalonSRX pivotMotor;
  //encoder
  //position coefficient
  //pid controller
  //offset?
  //forward and reverse limits
  //debug
  public Object pivotUp;

  /** Creates a new ExampleSubsystem. */
  public Intake() {//where you set it
    intakeBarMotor = new TalonSRX(9);
    pivotMotor = new TalonSRX(10);

    intakeBarMotor.configFactoryDefault();
    pivotMotor.configFactoryDefault();

    intakeBarMotor.setNeutralMode(NeutralMode.Coast);
    pivotMotor.setNeutralMode(NeutralMode.Brake);

    /*
    mShoulderLeftMotor.setInverted(true);

    mShoulderLeftMotor.setOpenLoopRampRate(0);
    mShoulderRightMotor.setOpenLoopRampRate(0);

    mShoulderEncoder = mShoulderLeftMotor.getEncoder();
    
    mShoulderPidController = mShoulderLeftMotor.getPIDController();
    mShoulderPidController.setFF(Constants.Arm.kShoulderFF, 0);
    mShoulderPidController.setP(Constants.Arm.kShoulderP, 0);
    mShoulderPidController.setI(Constants.Arm.kShoulderI, 0);
    mShoulderPidController.setD(Constants.Arm.kShoulderD, 0);
    mShoulderPidController.setIZone(Constants.Arm.kShoulderIz, 0);
    
    mShoulderPidController.setOutputRange(-1, 1);

    mShoulderPidController.setSmartMotionMaxVelocity(Constants.Arm.kShoulderMaxVel, 0);
    mShoulderPidController.setSmartMotionMaxAccel(Constants.Arm.kShoulderMaxAcc, 0);
    mShoulderPidController.setSmartMotionAllowedClosedLoopError(Constants.Arm.kShoulderAllowedErr, 0);
    mShoulderPidController.setSmartMotionMinOutputVelocity(Constants.Arm.kShoulderMinVel, 0);

    // Zero the motors
    rezeroMotors();

    // We configure the should limits after it is zero so we have accurate values.
    mShoulderForwardLimit = (float)calcShoulderPosFromAngle(Constants.Arm.kShoulderForwardLimitDeg);
    mShoulderReverseLimit = (float)calcShoulderPosFromAngle(Constants.Arm.kShoulderReverseLimitDeg);

    mShoulderLeftMotor.setSoftLimit(SoftLimitDirection.kForward, mShoulderForwardLimit);
    mShoulderLeftMotor.setSoftLimit(SoftLimitDirection.kReverse, mShoulderReverseLimit);
    mShoulderLeftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    mShoulderLeftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    */
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
          intakeBarMotor.set(ControlMode.PercentOutput, .8);
        },
        () -> {
            intakeBarMotor.set(ControlMode.PercentOutput, 0);
        }
        );
  }
  public CommandBase OuttakeCommand() {

  return startEnd(
    () -> {
      intakeBarMotor.set(ControlMode.PercentOutput, -.8);
    },
    () -> {
        intakeBarMotor.set(ControlMode.PercentOutput, 0);
    }
    );
  }

  public CommandBase testPivot(DoubleSupplier outputSupplier) {
    return runEnd(
    () -> {
      setOutput(outputSupplier.getAsDouble());
    }, 
    () -> {
      setOutput(0);
    }
    );
  }
  public void setOutput(double output) {
    pivotMotor.set(ControlMode.PercentOutput, output);
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
