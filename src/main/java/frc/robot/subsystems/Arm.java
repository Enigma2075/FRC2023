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

  private final CANSparkMax mElbowMotor;
  private final RelativeEncoder mElbowEncoder;

  private final double kShoulderPositionCoefficient = 2.0 * Math.PI / 1.0 * Constants.Arm.kShoulderReduction;
  private final double kElbowPositionCoefficient = 2.0 * Math.PI / 1.0 * Constants.Arm.kElbowReduction;

  private final SparkMaxPIDController mShoulderPidController;

  private Rotation2d mShoulderOffset;
  private Rotation2d mElbowOffset;

  private float mShoulderForwardLimit;
  private float mShoulderReverseLimit;

  private float mElbowForwardLimit;
  private float mElbowReverseLimit;

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
    mShoulderLeftMotor = new CANSparkMax(Constants.Arm.kShoulderLeftId, MotorType.kBrushless);
    mShoulderRightMotor = new CANSparkMax(Constants.Arm.kShoulderRightId, MotorType.kBrushless);

    mElbowMotor = new CANSparkMax(Constants.Arm.kElbowId, MotorType.kBrushless);

    mShoulderLeftMotor.restoreFactoryDefaults();
    mShoulderRightMotor.restoreFactoryDefaults();
    mElbowMotor.restoreFactoryDefaults();

    mElbowMotor.setIdleMode(IdleMode.kCoast);

    // 277.7
    //

    mShoulderLeftMotor.setIdleMode(IdleMode.kBrake);
    mShoulderRightMotor.setIdleMode(IdleMode.kBrake);

    mShoulderLeftMotor.setInverted(true);

    mElbowMotor.setOpenLoopRampRate(0);

    mShoulderLeftMotor.setOpenLoopRampRate(0);
    mShoulderRightMotor.setOpenLoopRampRate(0);

    // Configure Shoulder motors
    mShoulderRightMotor.follow(mShoulderLeftMotor, true);

    mShoulderEncoder = mShoulderLeftMotor.getEncoder();
    // mShoulderEncoder.setInverted(true);
    // mElbowEncoder.
    mElbowEncoder = mElbowMotor.getEncoder();

    mShoulderPidController = mShoulderLeftMotor.getPIDController();
    mShoulderPidController.setFF(Constants.Arm.kShoulderFF, 0);
    mShoulderPidController.setP(Constants.Arm.kShoulderP, 0);
    mShoulderPidController.setI(Constants.Arm.kShoulderI, 0);
    mShoulderPidController.setD(Constants.Arm.kShoulderD, 0);
    mShoulderPidController.setIZone(Constants.Arm.kShoulderIz, 0);
    
    mShoulderPidController.setSmartMotionMaxVelocity(Constants.Arm.kShoulderMaxVel, 0);
    mShoulderPidController.setSmartMotionMaxAccel(Constants.Arm.kShoulderMaxAcc, 0);
    mShoulderPidController.setSmartMotionAllowedClosedLoopError(Constants.Arm.kShoulderAllowedErr, 0);
    mShoulderPidController.setSmartMotionMinOutputVelocity(Constants.Arm.kShoulderMinVel, 0);

    // mStage1PidController.setReference

    rezeroMotors();

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

  //public Rotation2d getElbowCanCoderAngle() {
  //  return
  //    Rotation2d.fromDegrees(Cancoders.getInstance().getArmElbow().getAbsolutePosition());
  //}

  //public Rotation2d getAdjustedElbowCanCoderAngle() {
  //  return
  //    getElbowCanCoderAngle().rotateBy(Constants.Arm.kStage2Offset.inverse());
  //}

  public double calcShoulderPosFromAngle(double angle) {
    return ((Math.toRadians(angle) + mShoulderOffset.getRadians()) / kShoulderPositionCoefficient);
  }

  public void rezeroMotors() {
    // TODO: update this to work with a can coder so we are always starting at the right spot.
    // With this code we will ALWAYS have to have the Elbow inline with the Shoulder when the robot starts (restarting robot code should reset this)
    //mElbowOffset = Rotation2d.fromRadians(mElbowEncoder.getPosition() * kElbowPositionCoefficient)
    // .rotateBy(getAdjustedElbowCanCoderAngle().inverse());
    mElbowOffset = Rotation2d.fromRadians(mElbowEncoder.getPosition() * kElbowPositionCoefficient)
     .rotateBy(Rotation2d.fromRadians(mElbowEncoder.getPosition() * kElbowPositionCoefficient).inverse());

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

  public void setElbowVelocity(double vel) {
    mElbowMotor.set(vel);
  }

  public void setShoulderVelocity(double vel) {
    mPeriodicIO.shoulderTarget = vel * Constants.Arm.kShoulderMaxRPM;
    mShoulderPidController.setReference(mPeriodicIO.shoulderTarget, ControlType.kVelocity);
    //mShoulderLeftMotor.set(vel);
  }

  public void setShoulderAngle(double angle) {
    mPeriodicIO.shoulderTarget = calcShoulderPosFromAngle(angle);
    mShoulderPidController.setReference(mPeriodicIO.shoulderTarget, ControlType.kSmartMotion);
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
 
    if(mShoulderDebug) {
      mPeriodicIO.shoulderRightRot = mShoulderRightMotor.getEncoder().getPosition();
      
      mPeriodicIO.shoulderDeg = getShoulderAngle().getDegrees();
      mPeriodicIO.shoulderAbs = Cancoders.getInstance().getArmShoulder().getAbsolutePosition();

      mPeriodicIO.shoulderVel = mShoulderEncoder.getVelocity();
      mPeriodicIO.shoulderAppliedOutput = mShoulderLeftMotor.getAppliedOutput();
    }

    // Elbow
    mPeriodicIO.elbowRot = mElbowEncoder.getPosition();

    if(mElbowDebug)  {
      mPeriodicIO.elbowDeg = getShoulderAngle().getDegrees();
      //mPeriodicIO.elbowAbs = Cancoders.getInstance().getArmShoulder().getAbsolutePosition();

      mPeriodicIO.elbowVel = mElbowEncoder.getVelocity();
      mPeriodicIO.elbowAppliedOutput = mElbowMotor.getAppliedOutput();
    }
  }

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
      

      SmartDashboard.putNumber("E Deg", mPeriodicIO.elbowDeg);
      SmartDashboard.putNumber("E Rot", mPeriodicIO.elbowRot);
    }
  }
}