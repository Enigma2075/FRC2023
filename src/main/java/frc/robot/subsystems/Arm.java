// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.geometry.Rotation2d;
import frc.lib.other.Subsystem;
import frc.robot.Constants;

public class Arm extends Subsystem {
  private final CANSparkMax mShoulderRightMotor;
  private final CANSparkMax mShoulderLeftMotor;
  private final RelativeEncoder mShoulderEncoder;

  private final CANSparkMax mGrabberMotor;

  private final CANSparkMax mElbowMotor;
  private final RelativeEncoder mElbowEncoder;

  private final double kShoulderPositionCoefficient = 2.0 * Math.PI / 1.0 * Constants.Arm.kShoulderReduction;
  private final double kElbowPositionCoefficient = 2.0 * Math.PI / 1.0 * Constants.Arm.kElbowReduction;

  private final SparkMaxPIDController mShoulderPidController;
  private final SparkMaxPIDController mElbowPidController;

  private Rotation2d mShoulderOffset;
  private Rotation2d mElbowOffset;

  private float mShoulderForwardLimit;
  private float mShoulderReverseLimit;

  // TODO: need to figure out if we need/want these.
  //private float mElbowForwardLimit;
  //private float mElbowReverseLimit;

  private boolean mShoulderDebug = false;
  private boolean mElbowDebug = true;

  public static class PeriodicIO {
    // Shoulder
    double shoulderRightRot;
    double shoulderLeftRot;

    double shoulderAbs;
    double shoulderDeg;

    double shoulderVel;
    double shoulderTarget;
    double shoulderAppliedOutput;

    // Elbow
    double elbowAbs;    
    double elbowRot;

    double elbowAppliedOutput;
    double elbowVel;
    double elbowTarget;
    double elbowDeg;
  }

  private final PeriodicIO mPeriodicIO = new PeriodicIO();

  public Arm() {
    // Configure Shoulder motors
    mShoulderLeftMotor = new CANSparkMax(Constants.Arm.kShoulderLeftId, MotorType.kBrushless);
    mShoulderRightMotor = new CANSparkMax(Constants.Arm.kShoulderRightId, MotorType.kBrushless);

    mShoulderLeftMotor.restoreFactoryDefaults();
    mShoulderRightMotor.restoreFactoryDefaults();

    mShoulderLeftMotor.setIdleMode(IdleMode.kBrake);
    mShoulderRightMotor.setIdleMode(IdleMode.kBrake);

    mShoulderLeftMotor.setInverted(true);

    mShoulderLeftMotor.setOpenLoopRampRate(0);
    mShoulderRightMotor.setOpenLoopRampRate(0);

    mShoulderRightMotor.follow(mShoulderLeftMotor, true);

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

    // Configure Elbow motor
    mElbowMotor = new CANSparkMax(Constants.Arm.kElbowId, MotorType.kBrushless);
    mElbowMotor.restoreFactoryDefaults();

    mElbowMotor.setInverted(true);

    mElbowMotor.setIdleMode(IdleMode.kBrake);

    mElbowMotor.setOpenLoopRampRate(0);

    mElbowEncoder = mElbowMotor.getEncoder();

    mElbowPidController = mElbowMotor.getPIDController();
    mElbowPidController.setFF(Constants.Arm.kElbowFF, 0);
    mElbowPidController.setP(Constants.Arm.kElbowP, 0);
    mElbowPidController.setI(Constants.Arm.kElbowI, 0);
    mElbowPidController.setD(Constants.Arm.kElbowD, 0);
    mElbowPidController.setIZone(Constants.Arm.kElbowIz, 0);
    
    mElbowPidController.setSmartMotionMaxVelocity(Constants.Arm.kElbowMaxVel, 0);
    mElbowPidController.setSmartMotionMaxAccel(Constants.Arm.kElbowMaxAcc, 0);
    mElbowPidController.setSmartMotionAllowedClosedLoopError(Constants.Arm.kElbowAllowedErr, 0);
    mElbowPidController.setSmartMotionMinOutputVelocity(Constants.Arm.kElbowMinVel, 0);

    mGrabberMotor = new CANSparkMax(9, MotorType.kBrushless);

    // Zero the motors
    rezeroMotors();

    // We configure the should limits after it is zero so we have accurate values.
    mShoulderForwardLimit = (float)calcShoulderPosFromAngle(Constants.Arm.kShoulderForwardLimitDeg);
    mShoulderReverseLimit = (float)calcShoulderPosFromAngle(Constants.Arm.kShoulderReverseLimitDeg);

    mShoulderLeftMotor.setSoftLimit(SoftLimitDirection.kForward, mShoulderForwardLimit);
    mShoulderLeftMotor.setSoftLimit(SoftLimitDirection.kReverse, mShoulderReverseLimit);
    mShoulderLeftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    mShoulderLeftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public Rotation2d getShoulderAngle() {
    return Rotation2d.fromRadians(getUnclampedShoulderAngleRadians());
  }

  public double getUnclampedShoulderAngleRadians() {
    return (mShoulderEncoder.getPosition() * kShoulderPositionCoefficient) - mShoulderOffset.getRadians();
  }

  public Rotation2d getShoulderCanCoderAngle() {
    return Rotation2d.fromDegrees(Cancoders.getInstance().getArmShoulder().getAbsolutePosition());
  }

  public Rotation2d getAdjustedShoulderCanCoderAngle() {
    return getShoulderCanCoderAngle().rotateBy(Constants.Arm.kShoulderOffset.inverse());
  }

  public Rotation2d getElbowAngle() {
    return Rotation2d.fromRadians(getUnclampedElbowAngleRadians());
  }

  public double getUnclampedElbowAngleRadians() {
    return (mElbowEncoder.getPosition() * kElbowPositionCoefficient) - mElbowOffset.getRadians();
  }

  //public Rotation2d getElbowCanCoderAngle() {
  //  return
  //    Rotation2d.fromDegrees(Cancoders.getInstance().getArmElbow().getAbsolutePosition());
  //}

  //public Rotation2d getAdjustedElbowCanCoderAngle() {
  //  return
  //    getElbowCanCoderAngle().rotateBy(Constants.Arm.kStage2Offset.inverse());
  //}

  public double calcShoulderArbFF() {
    double cos = Math.cos(Math.toRadians(90.0 - Math.abs(mPeriodicIO.shoulderDeg)));
    double scaleCos = cos / Constants.Arm.kShoulderMaxCos;
    return scaleCos * Constants.Arm.kShoulderMaxArbFF * Math.signum(mPeriodicIO.shoulderDeg) * -1;
  }

  public double calcShoulderPosFromAngle(double angle) {
    return ((Math.toRadians(angle) + mShoulderOffset.getRadians()) / kShoulderPositionCoefficient);
  }

  public double calcElbowArbFF() {
    double angle = mPeriodicIO.elbowDeg + (mPeriodicIO.shoulderDeg * - 1);
    double cos = Math.cos(Math.toRadians(90 - Math.abs(angle)));
    return cos * Constants.Arm.kElbowMaxArbFF * Math.signum(angle);
  }

  public double calcElbowPosFromAngle(double angle) {
    return ((Math.toRadians(angle) + mElbowOffset.getRadians()) / kElbowPositionCoefficient);
  }

  public void rezeroMotors() {
    // TODO: update this to work with a can coder so we are always starting at the right spot.
    // With this code we will ALWAYS have to have the Elbow inline with the Shoulder when the robot starts (restarting robot code should reset this)
    //mElbowOffset = Rotation2d.fromRadians(mElbowEncoder.getPosition() * kElbowPositionCoefficient)
    // .rotateBy(getAdjustedElbowCanCoderAngle().inverse());
    mElbowOffset = Rotation2d.fromRadians(mElbowEncoder.getPosition() * kElbowPositionCoefficient)
     ;

    mShoulderOffset = Rotation2d.fromRadians(mShoulderEncoder.getPosition() * kShoulderPositionCoefficient)
        .rotateBy(getAdjustedShoulderCanCoderAngle().inverse());
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void setElbowOutput(double output) {
    mElbowMotor.set(output);
  }

  public void setElbowVelocity(double vel) {
    mPeriodicIO.elbowTarget = vel * Constants.Arm.kElbowMaxRPM;
    mElbowPidController.setReference(mPeriodicIO.elbowTarget, ControlType.kVelocity, 0, calcElbowArbFF(), ArbFFUnits.kPercentOut);
  }

  public void setElbowAngle(double angle) {
    mPeriodicIO.elbowTarget = calcElbowPosFromAngle(angle);
    mElbowPidController.setReference(mPeriodicIO.elbowTarget, ControlType.kSmartMotion, 0, calcShoulderArbFF(), ArbFFUnits.kPercentOut);
  }

  public void setShoulderOutput(double output) {
    mShoulderLeftMotor.set(output);
  }

  public void setShoulderVelocity(double vel) {
    mPeriodicIO.shoulderTarget = vel * Constants.Arm.kShoulderMaxRPM;
    mShoulderPidController.setReference(mPeriodicIO.shoulderTarget, ControlType.kVelocity, 0, calcShoulderArbFF(), ArbFFUnits.kPercentOut);
  }

  public void setShoulderAngle(double angle) {
    mPeriodicIO.shoulderTarget = calcShoulderPosFromAngle(angle);
    mShoulderPidController.setReference(mPeriodicIO.shoulderTarget, ControlType.kSmartMotion, 0, calcShoulderArbFF(), ArbFFUnits.kPercentOut);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public synchronized void readPeriodicInputs() {
    // Shoulder
    mPeriodicIO.shoulderLeftRot = mShoulderEncoder.getPosition();
    mPeriodicIO.shoulderDeg = getShoulderAngle().getDegrees();
      
    if(mShoulderDebug) {
      mPeriodicIO.shoulderRightRot = mShoulderRightMotor.getEncoder().getPosition();
      
      mPeriodicIO.shoulderAbs = Cancoders.getInstance().getArmShoulder().getAbsolutePosition();

      mPeriodicIO.shoulderVel = mShoulderEncoder.getVelocity();
      mPeriodicIO.shoulderAppliedOutput = mShoulderLeftMotor.getAppliedOutput();
    }

    // Elbow
    mPeriodicIO.elbowRot = mElbowEncoder.getPosition();
    mPeriodicIO.elbowDeg = getElbowAngle().getDegrees();
      
    if(mElbowDebug)  {
      //mPeriodicIO.elbowAbs = Cancoders.getInstance().getArmShoulder().getAbsolutePosition();

      mPeriodicIO.elbowVel = mElbowEncoder.getVelocity();
      mPeriodicIO.elbowAppliedOutput = mElbowMotor.getAppliedOutput();
    }
  }
/* 
  public CommandBase Grabber() {
    mGrabberMotor.set(ControlMode.PercentOutput, .8);
  }
*/
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void stop() {
    mElbowMotor.set(0);
    mShoulderRightMotor.set(0);
    mShoulderLeftMotor.set(0);
  }

  @Override
  public boolean checkSystem() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void outputTelemetry() {
    if(mShoulderDebug) {
      //SmartDashboard.putNumber("S F-Lim", mShoulderForwardLimit);
      //SmartDashboard.putNumber("S R-Lim", mShoulderReverseLimit);
      
      SmartDashboard.putNumber("S AV", mPeriodicIO.shoulderAppliedOutput);
      SmartDashboard.putNumber("S Vel", mPeriodicIO.shoulderVel);
      SmartDashboard.putNumber("S Vel-T", mPeriodicIO.shoulderTarget);
      SmartDashboard.putNumber("S Abs Deg", mPeriodicIO.shoulderAbs);
      
      SmartDashboard.putNumber("S Vel-Err", mPeriodicIO.shoulderTarget - mPeriodicIO.shoulderVel);

      SmartDashboard.putNumber("S Deg", mPeriodicIO.shoulderDeg);
      SmartDashboard.putNumber("S-R Rot", mPeriodicIO.shoulderRightRot);
      SmartDashboard.putNumber("S-L Rot", mPeriodicIO.shoulderLeftRot);
    }

    if(mElbowDebug) {
      //SmartDashboard.putNumber("E F-Lim", mElbowForwardLimit);
      //SmartDashboard.putNumber("E R-Lim", mElbowReverseLimit);
      
      SmartDashboard.putNumber("E AV", mPeriodicIO.elbowAppliedOutput);
      SmartDashboard.putNumber("E Vel", mPeriodicIO.elbowVel);
      SmartDashboard.putNumber("E Vel-T", mPeriodicIO.elbowTarget);
      SmartDashboard.putNumber("E Abs Deg", mPeriodicIO.elbowAbs);
      
      SmartDashboard.putNumber("E Vel-Err", mPeriodicIO.elbowTarget - mPeriodicIO.elbowVel);

      SmartDashboard.putNumber("E Deg", mPeriodicIO.elbowDeg);
      SmartDashboard.putNumber("E Rot", mPeriodicIO.elbowRot);
    
      SmartDashboard.putNumber("E Rel Deg", mPeriodicIO.elbowDeg + (mPeriodicIO.shoulderDeg * -1 ));
    }
  }
}
