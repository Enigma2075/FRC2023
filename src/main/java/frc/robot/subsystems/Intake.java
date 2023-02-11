package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.geometry.Rotation2d;
import frc.robot.Constants;

public class Intake extends SubsystemBase {//swhere you make it
  private final TalonSRX intakeMotor;
  private final TalonSRX pivotMotor;

  private final double kPivotPositionCoefficient = 2.0 * Math.PI / 4096 * Constants.Intake.kPivotReduction;
  
  /** Creates a new ExampleSubsystem. */
  public Intake() {
    // Pivot Motor
    pivotMotor = new TalonSRX(10);

    pivotMotor.configFactoryDefault();

    pivotMotor.setNeutralMode(NeutralMode.Brake);

    pivotMotor.setInverted(InvertType.InvertMotorOutput);

    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    pivotMotor.setSensorPhase(true);
    pivotMotor.setSelectedSensorPosition(Math.toRadians(-130) / kPivotPositionCoefficient);
    
    pivotMotor.configMotionAcceleration(Constants.Intake.kPivotAcc, 10);
		pivotMotor.configMotionCruiseVelocity(Constants.Intake.kPivotCruiseVel, 10);

    pivotMotor.config_kP(0, Constants.Intake.kPivotP, 10);
		pivotMotor.config_kI(0, Constants.Intake.kPivotI, 10);
		pivotMotor.config_kD(0, Constants.Intake.kPivotD, 10);
		pivotMotor.config_kF(0, Constants.Intake.kPivotFF, 10);
		pivotMotor.config_IntegralZone(0, Constants.Intake.kPivotIz, 10);
		pivotMotor.configClosedLoopPeakOutput(0, 1, 10);
		
    pivotMotor.configClosedLoopPeriod(0, 1, 10);
		
    // Intake Motor
    intakeMotor = new TalonSRX(9);

    intakeMotor.configFactoryDefault();

    intakeMotor.setNeutralMode(NeutralMode.Coast);

    intakeMotor.setInverted(InvertType.InvertMotorOutput);
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
          intakeMotor.set(ControlMode.PercentOutput, .9);
        },
        () -> {
            intakeMotor.set(ControlMode.PercentOutput, 0);
        }
        );
  }
  public CommandBase OuttakeCommand() {

  return startEnd(
    () -> {
      intakeMotor.set(ControlMode.PercentOutput, -.3);
    },
    () -> {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
    );
  }

  public CommandBase testPivot(DoubleSupplier intakeSupplier, DoubleSupplier outakeSupplier) {
    return runEnd(
    () -> {
      if(intakeSupplier.getAsDouble() > .8) {
        setPivotAngle(-10);
        intakeMotor.set(ControlMode.PercentOutput, .9);
      }
      else if(outakeSupplier.getAsDouble() > .8) {
        intakeMotor.set(ControlMode.PercentOutput, -.9);
      }
      else {
        setPivotAngle(-130);
        intakeMotor.set(ControlMode.PercentOutput, 0);
      }
      //setPivotOutput(outputSupplier.getAsDouble());
      //setPivotOutput(-.90);
    }, 
    () -> {
    }
    );
  }

  public double calcPivotArbFF() {
    double angle = getPivotAngle().getDegrees();
    double sign = angle > -90 ? -1 : 1;
    double cos = Math.cos(Math.toRadians(Math.abs(angle)));
    return cos * Constants.Intake.kPivotMaxArbFF * sign;
  }

  public double calcPivotPosFromAngle(double angle) {
    return ((Math.toRadians(angle)) / kPivotPositionCoefficient);
  }

  public void setPivotOutput(double output) {
    pivotMotor.set(ControlMode.PercentOutput, output);
  }
  
  public void setPivotVelocity(double vel) {
    pivotMotor.set(ControlMode.Velocity, vel * Constants.Intake.kPivotCruiseVel, DemandType.ArbitraryFeedForward, calcPivotArbFF());
  }

  public void setPivotAngle(double angle) {
    pivotMotor.set(ControlMode.MotionMagic, calcPivotPosFromAngle(angle), DemandType.ArbitraryFeedForward, calcPivotArbFF());
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
  
  public Rotation2d getPivotAngle() {
    return Rotation2d.fromRadians(getUnclampedPivotAngleRadians());
  }

  public double getUnclampedPivotAngleRadians() {
    return (pivotMotor.getSelectedSensorPosition() * kPivotPositionCoefficient);
  }

  // public Rotation2d getPivotCanCoderAngle() {
  //   return Rotation2d.fromDegrees(Cancoders.getInstance().getArmShoulder().getAbsolutePosition());
  // }

  // public Rotation2d getAdjustedShoulderCanCoderAngle() {
  //   return getShoulderCanCoderAngle().rotateBy(Constants.Arm.kShoulderOffset.inverse());
  // }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("IP Deg", getPivotAngle().getDegrees());

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
