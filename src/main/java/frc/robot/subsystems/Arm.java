// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import org.opencv.core.Point;

import com.ctre.phoenix.CANifier;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.geometry.Rotation2d;
import frc.lib.other.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.ArmMoveToScoreCommand;

public class Arm extends Subsystem {
  public enum ScoreMode {
    LOW,
    MIDDLE,
    HIGH
  }

  public enum ArmPosition {
    HANDOFF_CONE1(0, 55, 0, 55),
    HANDOFF_CONE2(5, 55, 5, 55),
    HANDOFF_CONE3(21, 43),
    HANDOFF_CONE4(0, 50),
    HANDOFF_CUBE(35, 47, 36, 40),
    // INTAKE_CONE(1.70, 46.22),
    // GRAB_CONE(18.32, 46.22),
    START(24, -5),
    DEFAULT(0, -2),
    DEFAULT_SHOULDER(0, Double.MIN_VALUE),
    DEFAULT_ELBOW(Double.MIN_VALUE, -2),
    HIGH_CONE(-29, -146, -29, -146),
    MEDIUM_CONE(-10, -92),
    HIGH_CUBE(-7, -95),
    HIGH_CUBE_AUTO(-8, -100),
    MEDIUM_CUBE(15, -60),
    MEDIUM_CUBE_AUTO(10, -65),
    // MEDIUM_AUTO_START(0, -70),
    // HAND_OFF(0, 50),
    SCORE_OFFSET_CONE_MID(Double.MIN_VALUE, 25, true),
    SCORE_OFFSET_CONE_HIGH(Double.MIN_VALUE, 15, true),
    SCORE_OFFSET_CONE_HIGH_AUTO(Double.MIN_VALUE, 18, true),
    HOLD(24, -5),
    SHELF(15, -66),
    AUTO_DROP(0, -10),
    FEEDER_CUBE(0, 55),
    LOW(24, -25);

    public final double mShoulderAngle;
    public final double mElbowAngle;
    public final boolean mIsOffset;

    private ArmPosition(double shoulderAngle, double elbowAngle) {
      this(shoulderAngle, elbowAngle, false);
    }

    private ArmPosition(double shoulderAngle, double elbowAngle, boolean isOffset) {
      this.mIsOffset = isOffset;
      this.mShoulderAngle = shoulderAngle;
      this.mElbowAngle = elbowAngle;
    }

    private ArmPosition(double shoulderAngle, double elbowAngle, double compShoulderAngle, double compElbowAngle) {
      this(shoulderAngle, elbowAngle, compShoulderAngle, compElbowAngle, false);
    }

    private ArmPosition(double shoulderAngle, double elbowAngle, double compShoulderAngle, double compElbowAngle,
        boolean isOffset) {
      if (!Constants.Robot.kPracticeBot) {
        this.mIsOffset = isOffset;
        this.mShoulderAngle = compShoulderAngle;
        this.mElbowAngle = compElbowAngle;
      } else {
        this.mIsOffset = isOffset;
        this.mShoulderAngle = shoulderAngle;
        this.mElbowAngle = elbowAngle;
      }
    }
  }

  private final RobotState mRobotState;

  private final CANSparkMax mShoulderRightMotor;
  private final CANSparkMax mShoulderLeftMotor;
  private final RelativeEncoder mShoulderEncoder;

  private final CANSparkMax mHandMotor;

  private final CANSparkMax mElbowMotor;
  private final RelativeEncoder mElbowEncoder;

  private final double kShoulderPositionCoefficient = 2.0 * Math.PI / 1.0 * Constants.Arm.kShoulderReduction;
  private final double kElbowPositionCoefficient = 2.0 * Math.PI / 1.0 * Constants.Arm.kElbowReduction;

  private final SparkMaxPIDController mShoulderPidController;
  private final SparkMaxPIDController mElbowPidController;

  private final TimeOfFlight mTimeOfFlight;

  private Rotation2d mShoulderOffset;
  private Rotation2d mElbowOffset;

  private float mShoulderForwardLimit;
  private float mShoulderReverseLimit;

  private float mElbowForwardLimit;
  private float mElbowReverseLimit;

  private Intake mIntake;

  // TODO: need to figure out if we need/want these.
  // private float mElbowForwardLimit;
  // private float mElbowReverseLimit;

  public static class PeriodicIO {
    ArmMotion[] sequence;
    int sequenceIndex = 0;

    ArmPosition targetPosition;

    Point armPosition;
    Point armTargetPosition;

    boolean force = false;

    // Shoulder
    double shoulderTarget = 0;
    double shoulderTargetCalc = 0;

    double shoulderRightRot;
    double shoulderLeftRot;

    double shoulderAbs;
    double shoulderDeg;

    double shoulderVel;
    double shoulderAppliedOutput;

    // Elbow
    double elbowTarget = 0;
    double elbowTargetCalc = 0;

    double elbowAbs;
    double elbowRot;

    double elbowAppliedOutput;
    double elbowVel;
    double elbowDeg;

    // Hand
    double handTarget = 0;
    double handCurrent;
    boolean handHasGamePeice = false;
    boolean handHasGamePeiceLast = false;
    // double handCurrentTime = Double.MIN_VALUE;
  }

  private final PeriodicIO mPeriodicIO = new PeriodicIO();

  public Arm(RobotState robotState) {
    mRobotState = robotState;

    // Configure Time Of Flight
    mTimeOfFlight = new TimeOfFlight(0);
    mTimeOfFlight.setRangeOfInterest(8, 8, 12, 12);

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

    mShoulderLeftMotor.setSmartCurrentLimit(40);
    mShoulderRightMotor.setSmartCurrentLimit(40);

    mShoulderRightMotor.follow(mShoulderLeftMotor, true);

    mShoulderRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    mShoulderRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    mShoulderRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
    mShoulderRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);
    mShoulderRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 200);

    mShoulderLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    mShoulderLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    mShoulderLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
    mShoulderLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);
    mShoulderLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 200);

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

    mElbowMotor.setInverted(false);

    mElbowMotor.setIdleMode(IdleMode.kBrake);

    mElbowMotor.setOpenLoopRampRate(0);
    mElbowMotor.setSmartCurrentLimit(40);

    mElbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    mElbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    mElbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
    mElbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);
    mElbowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 200);

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

    // Configure Hand motor
    mHandMotor = new CANSparkMax(Constants.Arm.kHandId, MotorType.kBrushless);
    mHandMotor.restoreFactoryDefaults();
    mHandMotor.setSmartCurrentLimit(60);

    mHandMotor.setInverted(false);

    mHandMotor.setIdleMode(IdleMode.kBrake);

    mHandMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    mHandMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    mHandMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
    mHandMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);
    mHandMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 200);

    // Zero the motor
    rezeroMotors();

    // We configure the should limits after it is zero so we have accurate values.
    mShoulderForwardLimit = (float) calcShoulderPosFromAngle(Constants.Arm.kShoulderForwardLimitDeg);
    mShoulderReverseLimit = (float) calcShoulderPosFromAngle(Constants.Arm.kShoulderReverseLimitDeg);

    mShoulderLeftMotor.setSoftLimit(SoftLimitDirection.kForward, mShoulderForwardLimit);
    mShoulderLeftMotor.setSoftLimit(SoftLimitDirection.kReverse, mShoulderReverseLimit);
    mShoulderLeftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    mShoulderLeftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    mElbowForwardLimit = (float) calcElbowPosFromAngle(150);
    mElbowReverseLimit = (float) calcElbowPosFromAngle(-150);

    mElbowMotor.setSoftLimit(SoftLimitDirection.kForward, mElbowForwardLimit);
    mElbowMotor.setSoftLimit(SoftLimitDirection.kReverse, mElbowReverseLimit);
    mElbowMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    mElbowMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    setHandOutput(.9);

    setPosition(ArmPosition.START);
    updatePositionSequence();
  }

  public void setIntake(Intake intake) {
    mIntake = intake;
  }

  public Point getArmPosition() {
    return mPeriodicIO.armPosition;
  }

  public Point getArmTargetPosition() {
    return mPeriodicIO.armTargetPosition;
  }

  private Point getArmPosition(double shoulderDeg, double elbowDeg) {
    double shoulderDegAdj = 90 + shoulderDeg;
    double elbowDegAdj = 180 - elbowDeg;

    double combinedRad = Math.toRadians(shoulderDegAdj + elbowDegAdj);
    double shoulderRad = Math.toRadians(shoulderDegAdj);
    double x = (Math.cos(combinedRad) * (28.741 + 1)) + (Math.cos(shoulderRad) * 40.719);
    double y = (Math.sin(combinedRad) * (28.741 + 1)) + (Math.sin(shoulderRad) * 40.719);

    return new Point(x, y);
  }

  public boolean handHasGamePeice() {
    return mPeriodicIO.handHasGamePeice;
  }

  private boolean readHandHasGamePeice() {
    if (mTimeOfFlight.isRangeValid()) {
      if (mRobotState.isConeMode()) {
        mPeriodicIO.handHasGamePeiceLast = mTimeOfFlight.getRange() < 50;
      } else {
        mPeriodicIO.handHasGamePeiceLast = mTimeOfFlight.getRange() < 100;
      }
    }
    mRobotState.setHasGamePiece(mPeriodicIO.handHasGamePeiceLast);
    return mPeriodicIO.handHasGamePeiceLast;
  }

  public double getShoulderTarget() {
    return mPeriodicIO.shoulderTarget;
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

  public double getElbowTarget() {
    return mPeriodicIO.shoulderTarget;
  }

  public Rotation2d getElbowAngle() {
    return Rotation2d.fromRadians(getUnclampedElbowAngleRadians());
  }

  public double getUnclampedElbowAngleRadians() {
    return (mElbowEncoder.getPosition() * kElbowPositionCoefficient) - mElbowOffset.getRadians();
  }

  public Rotation2d getElbowCanCoderAngle() {
    return Rotation2d.fromDegrees(Cancoders.getInstance().getArmElbow().getAbsolutePosition());
  }

  public Rotation2d getAdjustedElbowCanCoderAngle() {
    return getElbowCanCoderAngle().rotateBy(Constants.Arm.kElbowOffset.inverse());
  }

  public double calcShoulderArbFF() {
    double cos = Math.cos(Math.toRadians(90.0 - Math.abs(mPeriodicIO.shoulderDeg)));
    double scaleCos = cos / Constants.Arm.kShoulderMaxCos;
    return scaleCos * Constants.Arm.kShoulderMaxArbFF * Math.signum(mPeriodicIO.shoulderDeg) * -1;
  }

  public double calcShoulderPosFromAngle(double angle) {
    return ((Math.toRadians(angle) + mShoulderOffset.getRadians()) / kShoulderPositionCoefficient);
  }

  public double calcElbowArbFF() {
    double angle = mPeriodicIO.elbowDeg + (mPeriodicIO.shoulderDeg * -1);
    double cos = Math.cos(Math.toRadians(90 - Math.abs(angle)));
    return cos * Constants.Arm.kElbowMaxArbFF * Math.signum(angle);
  }

  public double calcElbowPosFromAngle(double angle) {
    return ((Math.toRadians(angle) + mElbowOffset.getRadians()) / kElbowPositionCoefficient);
  }

  public boolean isConeMode() {
    return mRobotState.isConeMode();
  }

  public void rezeroMotors() {
    mElbowOffset = Rotation2d.fromRadians(mElbowEncoder.getPosition() *
        kElbowPositionCoefficient)
        .rotateBy(getAdjustedElbowCanCoderAngle().inverse());

    // With this code we will ALWAYS have to have the Elbow inline with the Shoulder
    // when the robot starts (restarting robot code should reset this)
    // mElbowOffset = Rotation2d.fromRadians(mElbowEncoder.getPosition() *
    // kElbowPositionCoefficient);

    mShoulderOffset = Rotation2d.fromRadians(mShoulderEncoder.getPosition() * kShoulderPositionCoefficient)
        .rotateBy(getAdjustedShoulderCanCoderAngle().inverse());
  }

  public CommandBase moveToScore(ScoreMode mode) {
    return new ArmMoveToScoreCommand(this, mRobotState, mode, false);
  }

  public ArmPosition getTargetPosition() {
    return mPeriodicIO.targetPosition;
  }

  public CommandBase handCommand() {
    return handCommand(true);
  }

  public CommandBase handCommand(boolean end) {
    return handCommand(end, .9);
  }

  public CommandBase handCommand(boolean end, double power) {
    return runEnd(
        () -> {
          mPeriodicIO.handTarget = power;
        },
        () -> {
          if (end) {
            mPeriodicIO.handTarget = 0;
          }
        });
  }

  public boolean atHold() {
    if (atPosition()) {
      if (mPeriodicIO.elbowTarget == ArmPosition.HOLD.mElbowAngle
          && mPeriodicIO.shoulderTarget == ArmPosition.HOLD.mShoulderAngle) {
        return true;
      }
    }
    return false;
  }

  public boolean goingToDefault() {
      if (mPeriodicIO.elbowTarget == ArmPosition.DEFAULT.mElbowAngle
          && mPeriodicIO.shoulderTarget == ArmPosition.DEFAULT.mShoulderAngle) {
        return true;
      }
    return false;
  }

  public boolean atDefault() {
    if (atPosition()) {
      if (goingToDefault()) {
        return true;
      }
    }
    return false;
  }

  public boolean atPosition() {
    return shoulderAtPosition() && elbowAtPosition();
  }

  public boolean shoulderAtPosition() {
    return Math.abs(mPeriodicIO.shoulderTarget - mPeriodicIO.shoulderDeg) < 2;
  }

  public boolean elbowAtPosition() {
    return Math.abs(mPeriodicIO.elbowTarget - mPeriodicIO.elbowDeg) < 2;
  }

  public boolean isSequenceComplete() {
    return isSequenceComplete(true);
  }

  public boolean isSequenceComplete(boolean wait) {
    if (wait) {
      return mPeriodicIO.sequenceIndex >= mPeriodicIO.sequence.length && atPosition();
    } else {
      return mPeriodicIO.sequenceIndex >= mPeriodicIO.sequence.length;
    }
  }

  public void setPosition(ArmPosition position) {
    setPositions(new ArmPosition[] { position });
  }

  public void setPosition(ArmMotion motion) {
    setPositions(new ArmMotion[] { motion });
  }

  public void setPositions(ArmPosition... positions) {
    setPositions(false, positions);
  }

  public void setPositions(boolean force, ArmPosition... positions) {
    mPeriodicIO.force = force;
    ArrayList<ArmMotion> motions = new ArrayList<ArmMotion>();

    for (ArmPosition position : positions) {
      motions.add(new ArmMotion(position));
    }

    mPeriodicIO.sequence = new ArmMotion[motions.size()];
    mPeriodicIO.sequence = motions.toArray(mPeriodicIO.sequence);
    mPeriodicIO.sequenceIndex = 0;
    updatePositionSequence();
  }

  public void setPositions(ArmMotion... motions) {
    mPeriodicIO.force = false;
    mPeriodicIO.sequence = motions;
    mPeriodicIO.sequenceIndex = 0;
    updatePositionSequence();
  }

  public void writeElbowOutput() {
    mPeriodicIO.elbowTargetCalc = mPeriodicIO.elbowTarget;
    mElbowMotor.set(mPeriodicIO.elbowTargetCalc);
  }

  public void writeElbowVelocity() {
    mPeriodicIO.elbowTargetCalc = mPeriodicIO.elbowTarget * Constants.Arm.kElbowMaxRPM;
    mElbowPidController.setReference(mPeriodicIO.elbowTargetCalc, ControlType.kVelocity, 0, calcElbowArbFF(),
        ArbFFUnits.kPercentOut);
  }

  public void writeElbowPosition() {
    int index = mPeriodicIO.sequenceIndex;
    if (index > 0) {
      index--;
    }

    if (!mPeriodicIO.sequence[index].checkElbowCondition(mPeriodicIO.shoulderDeg, mPeriodicIO.elbowDeg)) {
      return;
    }

    mPeriodicIO.elbowTargetCalc = calcElbowPosFromAngle(mPeriodicIO.elbowTarget);
    mElbowPidController.setReference(mPeriodicIO.elbowTargetCalc, ControlType.kSmartMotion, 0, calcShoulderArbFF(),
        ArbFFUnits.kPercentOut);
  }

  public void writeShoulderOutput() {
    mPeriodicIO.shoulderTargetCalc = mPeriodicIO.shoulderTarget;
    mShoulderLeftMotor.set(mPeriodicIO.shoulderTargetCalc);
  }

  public void writeShoulderVelocity() {
    if (mPeriodicIO.targetPosition.mElbowAngle != Double.MIN_VALUE) {
      mPeriodicIO.shoulderTargetCalc = mPeriodicIO.shoulderTarget * Constants.Arm.kShoulderMaxRPM;
    }
    mShoulderPidController.setReference(mPeriodicIO.shoulderTargetCalc, ControlType.kVelocity, 0, calcShoulderArbFF(),
        ArbFFUnits.kPercentOut);
  }

  public void writeShoulderPosition() {
    if (mPeriodicIO.force) {
      mPeriodicIO.shoulderTargetCalc = calcShoulderPosFromAngle(mPeriodicIO.shoulderTarget);
      mShoulderPidController.setReference(mPeriodicIO.shoulderTargetCalc, ControlType.kSmartMotion, 0,
          calcShoulderArbFF(), ArbFFUnits.kPercentOut);
      return;
    }

    int index = mPeriodicIO.sequenceIndex;
    if (index > 0) {
      index--;
    }

    if (!mPeriodicIO.sequence[index].checkShoulderCondition(mPeriodicIO.shoulderDeg, mPeriodicIO.elbowDeg)) {
      return;
    }

    if (mPeriodicIO.targetPosition.mShoulderAngle != Double.MIN_VALUE) {
      mPeriodicIO.shoulderTargetCalc = calcShoulderPosFromAngle(mPeriodicIO.shoulderTarget);
    }

    double finalTarget = mPeriodicIO.shoulderTargetCalc;

    Point intakePos = mIntake.getIntakePosition();
    Point intakeTargetPos = mIntake.getIntakeTargetPosition();

    Point armPos = getArmPosition();
    Point armTargetPos = getArmTargetPosition();

    // Intake is up and is staying up so we are good to do anything
    if (intakePos.x > -7 && intakeTargetPos.x > -7) {

    }
    // The intake is at the up right position so we are Okay...
    // TODO: need to add more logic to protect this position.
    else if (intakePos.x < -20 && intakePos.x > -25 && intakeTargetPos.x < -20 && intakeTargetPos.x > -25) {

    }
    // Intake is up and wants to go down and we want to move the arm out
    else if (intakeTargetPos.x < -7
        && armTargetPos.x < intakePos.x && armTargetPos.y < intakePos.y && armTargetPos.x < 0 && intakePos.x > armPos.x -5) {
      finalTarget = 0;
    }

    mShoulderPidController.setReference(finalTarget, ControlType.kSmartMotion, 0,
        calcShoulderArbFF(), ArbFFUnits.kPercentOut);
  }

  public void writeHandOutput() {
    if (mPeriodicIO.handTarget > 0) {
      if (mPeriodicIO.handHasGamePeice) {
        // if((atDefault() || atHold())) {
        if (mRobotState.isConeMode()) {
          mHandMotor.set(.04);
        } else {
          mHandMotor.set(.02);
        }
        // }
        // else {
        // mHandMotor.set(mPeriodicIO.handTarget * .8);
        // }
      } else {
        // if (mPeriodicIO.handCurrent > 10) {
        // if(mPeriodicIO.handCurrentTime == Double.MIN_VALUE) {
        // mPeriodicIO.handCurrentTime = Timer.getFPGATimestamp();
        // }
        // if(Timer.getFPGATimestamp() - mPeriodicIO.handCurrentTime > 1) {
        // mPeriodicIO.handHasGamePeice = true;
        // mHandMotor.set(.02);
        // }
        // else {
        // mHandMotor.set(mPeriodicIO.handTarget);
        // }
        // }
        // else {
        // mPeriodicIO.handCurrentTime = Double.MIN_VALUE;
        mHandMotor.set(mPeriodicIO.handTarget);
      }

    } else {
      mHandMotor.set(mPeriodicIO.handTarget);
    }
  }

  public void setHandOutput(double output) {
    mPeriodicIO.handTarget = output;
  }

  @Override
  public synchronized void readPeriodicInputs() {
    mPeriodicIO.handHasGamePeice = readHandHasGamePeice();

    // Shoulder
    mPeriodicIO.shoulderLeftRot = mShoulderEncoder.getPosition();
    mPeriodicIO.shoulderDeg = getShoulderAngle().getDegrees();

    if (Constants.Arm.kShoulderDebug) {
      mPeriodicIO.shoulderRightRot = mShoulderRightMotor.getEncoder().getPosition();

      mPeriodicIO.shoulderAbs = Cancoders.getInstance().getArmShoulder().getAbsolutePosition();

      mPeriodicIO.shoulderVel = mShoulderEncoder.getVelocity();
      mPeriodicIO.shoulderAppliedOutput = mShoulderLeftMotor.getAppliedOutput();
    }

    // Elbow
    mPeriodicIO.elbowRot = mElbowEncoder.getPosition();
    mPeriodicIO.elbowDeg = getElbowAngle().getDegrees();

    if (Constants.Arm.kElbowDebug) {
      mPeriodicIO.elbowAbs = Cancoders.getInstance().getArmElbow().getAbsolutePosition();

      mPeriodicIO.elbowVel = mElbowEncoder.getVelocity();
      mPeriodicIO.elbowAppliedOutput = mElbowMotor.getAppliedOutput();
    }

    // Hand
    if (Constants.Arm.kElbowDebug || Constants.Arm.kShoulderDebug) {
      mPeriodicIO.handCurrent = mHandMotor.getOutputCurrent();
    }

    mPeriodicIO.armPosition = getArmPosition(mPeriodicIO.shoulderDeg, mPeriodicIO.elbowDeg);
    mPeriodicIO.armTargetPosition = getArmPosition(mPeriodicIO.shoulderTarget, mPeriodicIO.elbowTarget);
  }

  private void updatePositionSequence() {
    if (isSequenceComplete()) {
      return;
    }

    ArmMotion motion;
    if (mPeriodicIO.sequenceIndex < mPeriodicIO.sequence.length) {
      motion = mPeriodicIO.sequence[mPeriodicIO.sequenceIndex];
    } else {
      motion = mPeriodicIO.sequence[mPeriodicIO.sequenceIndex - 1];
    }

    // if (motion.getPosition() == mPeriodicIO.targetPosition ) {
    //   if((mPeriodicIO.targetPosition.mElbowAngle != mPeriodicIO.elbowTarget && mPeriodicIO.targetPosition.mElbowAngle != Double.MIN_VALUE)
    //     || (mPeriodicIO.targetPosition.mShoulderAngle != mPeriodicIO.shoulderTarget && mPeriodicIO.targetPosition.mShoulderAngle != Double.MIN_VALUE)) {
    //       if (mPeriodicIO.targetPosition.mIsOffset) {
    //         if (mPeriodicIO.targetPosition.mElbowAngle != Double.MIN_VALUE) {
    //           mPeriodicIO.elbowTarget += mPeriodicIO.targetPosition.mElbowAngle;
    //         }
    //         if (mPeriodicIO.targetPosition.mShoulderAngle != Double.MIN_VALUE) {
    //           mPeriodicIO.shoulderTarget += mPeriodicIO.targetPosition.mShoulderAngle;
    //         }
    //       } else {
    //         if (mPeriodicIO.targetPosition.mElbowAngle != Double.MIN_VALUE) {
    //           mPeriodicIO.elbowTarget = mPeriodicIO.targetPosition.mElbowAngle;
    //         }
    //         if (mPeriodicIO.targetPosition.mShoulderAngle != Double.MIN_VALUE) {
    //           mPeriodicIO.shoulderTarget = mPeriodicIO.targetPosition.mShoulderAngle;
    //         }
    //       }
    //   }

    //   return;
    // }

    if (atPosition() && motion.getPosition() != mPeriodicIO.targetPosition) {
      ArmPosition position = mPeriodicIO.targetPosition = motion.getPosition();
      mPeriodicIO.sequenceIndex++;

      if (position.mIsOffset) {
        if (position.mElbowAngle != Double.MIN_VALUE) {
          mPeriodicIO.elbowTarget += position.mElbowAngle;
        }
        if (position.mShoulderAngle != Double.MIN_VALUE) {
          mPeriodicIO.shoulderTarget += position.mShoulderAngle;
        }
      } else {
        if (position.mElbowAngle != Double.MIN_VALUE) {
          mPeriodicIO.elbowTarget = position.mElbowAngle;
        }
        if (position.mShoulderAngle != Double.MIN_VALUE) {
          mPeriodicIO.shoulderTarget = position.mShoulderAngle;
        }
      }
    }
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    updatePositionSequence();
    switch (Constants.Arm.kShoulderMode) {
      case MOTION_MAGIC:
        writeShoulderPosition();
        writeElbowPosition();
        break;
      case VELOCITY:
        writeShoulderVelocity();
        writeElbowVelocity();
        break;
      case PERCENT_OUTPUT:
        writeShoulderOutput();
        writeElbowOutput();
    }
    writeHandOutput();
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
    //mElbowMotor.set(0);
    //mShoulderRightMotor.set(0);
    //mShoulderLeftMotor.set(0);
  }

  @Override
  public boolean checkSystem() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void outputTelemetry() {
    if (Constants.Arm.kShoulderDebug) {
      // SmartDashboard.putNumber("S F-Lim", mShoulderForwardLimit);
      // SmartDashboard.putNumber("S R-Lim", mShoulderReverseLimit);

      SmartDashboard.putNumber("S AV", mPeriodicIO.shoulderAppliedOutput);
      SmartDashboard.putNumber("S Vel", mPeriodicIO.shoulderVel);
      SmartDashboard.putNumber("S Vel-T", mPeriodicIO.shoulderTargetCalc);
      SmartDashboard.putNumber("S Abs Deg", mPeriodicIO.shoulderAbs);

      SmartDashboard.putNumber("S Vel-Err", mPeriodicIO.shoulderTargetCalc - mPeriodicIO.shoulderVel);

      SmartDashboard.putNumber("S Deg", mPeriodicIO.shoulderDeg);
      SmartDashboard.putNumber("S-R Rot", mPeriodicIO.shoulderRightRot);
      SmartDashboard.putNumber("S-L Rot", mPeriodicIO.shoulderLeftRot);
      SmartDashboard.putBoolean("S AtPos", shoulderAtPosition());
      SmartDashboard.putNumber("S Pos-T", mPeriodicIO.shoulderTarget);
    }

    if (Constants.Arm.kElbowDebug) {
      // SmartDashboard.putNumber("E F-Lim", mElbowForwardLimit);
      // SmartDashboard.putNumber("E R-Lim", mElbowReverseLimit);

      SmartDashboard.putNumber("E AV", mPeriodicIO.elbowAppliedOutput);
      SmartDashboard.putNumber("E Vel", mPeriodicIO.elbowVel);
      SmartDashboard.putNumber("E Vel-T", mPeriodicIO.elbowTargetCalc);
      SmartDashboard.putNumber("E Abs Deg", mPeriodicIO.elbowAbs);

      SmartDashboard.putNumber("E Vel-Err", mPeriodicIO.elbowTargetCalc - mPeriodicIO.elbowVel);

      SmartDashboard.putNumber("E Deg", mPeriodicIO.elbowDeg);
      SmartDashboard.putNumber("E Rot", mPeriodicIO.elbowRot);

      SmartDashboard.putNumber("E Rel Deg", mPeriodicIO.elbowDeg + (mPeriodicIO.shoulderDeg * -1));
      SmartDashboard.putBoolean("E AtPos", elbowAtPosition());
      SmartDashboard.putNumber("E Pos-T", mPeriodicIO.elbowTarget);

      SmartDashboard.putNumber("H Current", mPeriodicIO.handCurrent);
      SmartDashboard.putNumber("H ToF", mTimeOfFlight.getRange());

      Point pos = getArmPosition();
      SmartDashboard.putString("A Pos", String.format("X:%f, Y:%f", pos.x, pos.y));

      Point targetPos = getArmTargetPosition();
      SmartDashboard.putString("A T-Pos", String.format("X:%f, Y:%f", targetPos.x, targetPos.y));
      SmartDashboard.putString("A PosName", mPeriodicIO.targetPosition.name());
    }
  }
}
