package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.opencv.core.Point;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.other.Subsystem;
import frc.robot.Constants;

public class Intake extends Subsystem {
  public enum PivotPosition {
    HANDOFF_CONE(-71.5, -68),
    HANDOFF_CUBE(0),
    FEEDER(-68),
    DOWN(6),
    UP(-125);

    public final double mAngle;

    private PivotPosition(double angle) {
      this.mAngle = angle;
    }

    private PivotPosition(double angle, double compAngle) {
      if(!Constants.Robot.kPracticeBot) {
        this.mAngle = compAngle;
      }
      else {
        this.mAngle = angle;
      }
    }
  }

  public enum IntakeMode {
    CONE_IN(.9),
    CONE_HANDOFF(-.02),
    CONE_OUT(-.9),
    CUBE_IN(-.5),
    CUBE_OUT(.9),
    CONE_HOLD(.02),
    CUBE_HOLD(.04),
    STOP(0);

    public final double mOutput;

    private IntakeMode(double output) {
      this.mOutput = output;
    }
  }

  private final CANSparkMax intakeMotor;
  private final TalonSRX pivotMotor;

  private final RobotState mRobotState;

  private final double kPivotPositionCoefficient = 2.0 * Math.PI / 4096.0 * Constants.Intake.kPivotReduction;
  private final double kPivotAbsPositionCoefficient = 2.0 * Math.PI / 1 * Constants.Intake.kPivotReduction;

  //private Rotation2d mPivotOffset;

  private Arm mArm;

  private static boolean mPivotReset = false;

  public static class PeriodicIO {
    // Pivot
    PivotPosition pivotPosition = PivotPosition.UP;
    double pivotTarget;
    Rotation2d pivotAngle;
    double pivotAbs;

    // Intake
    IntakeMode intakeMode = IntakeMode.STOP;
  }

  private final PeriodicIO mPeriodicIO = new PeriodicIO();

  /** Creates a new ExampleSubsystem. */
  public Intake(RobotState robotState) {
    mRobotState = robotState;

    // Pivot Motor
    pivotMotor = new TalonSRX(Constants.Intake.kPivotId);

    pivotMotor.configFactoryDefault();

    pivotMotor.setNeutralMode(NeutralMode.Brake);

    pivotMotor.setInverted(InvertType.None);

    pivotMotor.configFeedbackNotContinuous(true, 10);

    //pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, 10);
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    pivotMotor.setSensorPhase(true);

    // double pivotAbsPos = pivotMotor.getSelectedSensorPosition(1);
    // We want the encoder counts for all the way up to be 118 deg
    // mPivotOffset = pivotAbsPos > Constants.Intake.kPivotOffset + 100
    // ? (4096 - pivotAbsPos + Constants.Intake.kPivotOffset)
    // : (Constants.Intake.kPivotOffset - pivotAbsPos);
    // mPivotOffset = (Math.toRadians(118) / kPivotPositionCoefficient) -
    // mPivotOffset;
    pivotMotor.setSelectedSensorPosition(0);

    pivotMotor.configMotionAcceleration(Constants.Intake.kPivotAcc, 10);
    pivotMotor.configMotionCruiseVelocity(Constants.Intake.kPivotCruiseVel, 10);

    pivotMotor.config_kP(0, Constants.Intake.kPivotP, 10);
    pivotMotor.config_kI(0, Constants.Intake.kPivotI, 10);
    pivotMotor.config_kD(0, Constants.Intake.kPivotD, 10);
    pivotMotor.config_kF(0, Constants.Intake.kPivotFF, 10);
    pivotMotor.config_IntegralZone(0, Constants.Intake.kPivotIz, 10);
    pivotMotor.configClosedLoopPeakOutput(0, 1, 10);

    pivotMotor.configClosedLoopPeriod(0, 1, 10);

    pivotMotor.configPeakCurrentLimit(40, 10);
    pivotMotor.configPeakCurrentDuration(50, 10);
    pivotMotor.configContinuousCurrentLimit(0, 10);
    pivotMotor.enableCurrentLimit(true);

    pivotMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
    5, 10);

    pivotMotor.configNeutralDeadband(0);

    // Intake Motor
    intakeMotor = new CANSparkMax(Constants.Intake.kIntakeId, MotorType.kBrushless);

    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setIdleMode(IdleMode.kCoast);

    intakeMotor.setInverted(true);
    //intakeMotor.setOpenLoopRampRate(1);
    intakeMotor.setSmartCurrentLimit(50);

    zeroMotors();
  }

  public void setArm(Arm arm) {
    mArm = arm;
  }

  public void setPivotPosition(PivotPosition position) {
    mPeriodicIO.pivotPosition = position;
    mPeriodicIO.pivotTarget = position.mAngle;
  }

  public void setIntake(IntakeMode mode) {
    mPeriodicIO.intakeMode = mode;
  }

  public Rotation2d getPivotCanCoderAngle() {
    return Rotation2d.fromDegrees(Cancoders.getInstance().getIntakePivot().getAbsolutePosition());
  }

  public double getAdjustedPivotCanCoderAngle() {
    Rotation2d raw = getPivotCanCoderAngle();
    if(raw.getDegrees() < Constants.Intake.kPivotOffset.getDegrees()) {
      return (Constants.Intake.kPivotOffset.getDegrees() - raw.getDegrees()) * -1;
    }
    else {
      return (360.0 - raw.getDegrees() + Constants.Intake.kPivotOffset.getDegrees()) * -1;
    }
  }

  private void zeroMotors() {
    //Rotation2d zeroOffset = getAdjustedPivotCanCoderAngle();

    //if(zeroOffset.getRadians() < 0) {
    //  zeroOffset = Rotation2d.fromRadians(kPivotPositionCoefficient);
    //}

    double startingPosition = Math.toRadians(getAdjustedPivotCanCoderAngle() + 14) / kPivotAbsPositionCoefficient / kPivotPositionCoefficient;

    //if(startingPosition < 0) {
      
    //}
    pivotMotor.setSelectedSensorPosition(startingPosition);


    //mPivotOffset = Rotation2d.fromRadians(pivotMotor.getSelectedSensorPosition() *
    //kPivotPositionCoefficient)
    //.rotateBy(getAdjustedPivotCanCoderAngle().inverse());
  }

  public void writeIntake() {
    intakeMotor.set(mPeriodicIO.intakeMode.mOutput);
  }

  public void writePivot() {
    //if (mPivotReset) {
      switch (Constants.Intake.kPivotMode) {
        case MOTION_MAGIC:
          writePivotAngle(mPeriodicIO.pivotPosition.mAngle);
          break;
        case PERCENT_OUTPUT:
          writePivotOutput(mPeriodicIO.pivotTarget);
          break;
        case VELOCITY:
          writePivotVelocity(mPeriodicIO.pivotTarget);
          break;
      }
    //} else {
    //  if (Math.abs(pivotMotor.getStatorCurrent()) > 40) {
    //    writePivotOutput(0);
    //    mPivotReset = true;
    //    double startingPosition = (Math.toRadians(-118 - 16) / kPivotPositionCoefficient);
    //    pivotMotor.setSelectedSensorPosition(startingPosition);
    //  } else {
    //    writePivotOutput(-1);
    // }
    //}
  }

  public CommandBase autoCommand(PivotPosition pivot, IntakeMode intake) {
    return runOnce(
        () -> {
          setPivotPosition(pivot);
          setIntake(intake);
        });
  }

  public CommandBase setPivot(PivotPosition pivot) {
    return runOnce(
      () -> {
        setPivotPosition(pivot);
      }
    );
  }

  public CommandBase intakeFeederCommand() {
    return runEnd(() -> {
      setPivotPosition(PivotPosition.FEEDER);
      setIntake(IntakeMode.CONE_IN);
    },
    () -> {
      setIntake(IntakeMode.CONE_HOLD);
    });
  }

  public CommandBase intakeCommand() {
    return intakeCommand(false);
  }

  public CommandBase intakeCommand(boolean runOnce) {
   Runnable runnable = () -> {
          setPivotPosition(PivotPosition.DOWN);
          if(mRobotState.isCubeMode()) {
            setIntake(IntakeMode.CUBE_IN);
            }
          else {
            setIntake(IntakeMode.CONE_IN);
          }
        };
       
        if(runOnce) {
          return runOnce(runnable);
        }
        else {
          return runEnd(
            runnable,
            () -> {
              if(mRobotState.isCubeMode()) {
                setIntake(IntakeMode.CUBE_HOLD);
              }
              else {        
                setIntake(IntakeMode.CONE_HOLD);
              }  
            }
          );
        }
  }

  public CommandBase holdCommand() {
    return runOnce( () -> {
      if(mRobotState.isCubeMode()) {
        setIntake(IntakeMode.CUBE_HOLD);
      }
      else {
        setIntake(IntakeMode.CONE_HOLD);
      }
    });
  }

  public CommandBase outtakeCommand() {
    return runEnd(
        () -> {
          if(mRobotState.isCubeMode()) {
            setIntake(IntakeMode.CUBE_OUT);
          }
          else {
            setIntake(IntakeMode.CONE_OUT);
          }
        },
        () -> {
          setIntake(IntakeMode.STOP);
        });
  }

  // public CommandBase defaultCommand(DoubleSupplier intakeSupplier, DoubleSupplier outakeSupplier) {
  //   return runEnd(
  //       () -> {
  //         // if (outakeSupplier.getAsDouble() > .8) {
  //         // setPivotOutput(1);
  //         // } else if (intakeSupplier.getAsDouble() > .8) {
  //         // setPivotOutput(-1);
  //         // } else {
  //         // setPivotOutput(0);
  //         // }

  //         if (intakeSupplier.getAsDouble() > .8) {
  //           mPeriodicIO.mode = Mode.CONE;
  //           setPivotPosition(PivotPosition.DOWN);
  //           setIntake(IntakeMode.CONE_IN);
  //         } else if (outakeSupplier.getAsDouble() > .8) {
  //           mPeriodicIO.mode = Mode.CONE;
  //           setIntake(IntakeMode.CONE_IN);
  //         } else {
  //           switch(mPeriodicIO.mode) {
  //             case CONE:
  //               setPivotPosition(PivotPosition.UP);
  //               setIntake(IntakeMode.STOP);
  //               break;
  //             case CUBE:
  //               setPivotPosition(PivotPosition.UP);
  //               setIntake(IntakeMode.CUBE_HOLD);
  //               break;
  //           }
  //         }
  //       },
  //       () -> {
  //       });
  // }

  public CommandBase testPivotCommand(DoubleSupplier output) {
    return runEnd(
        () -> {
          setPivotOutput(output.getAsDouble());
        },
        () -> {
          setPivotOutput(0);
        });
  }

  public CommandBase testPivotFFCommand() {
    return runEnd(
        () -> {
          // setPivotOutput(Constants.Intake.kPivotMaxArbFF * -1);
          setPivotOutput(calcPivotArbFF());
        },
        () -> {
          setPivotOutput(0);
        });
  }
  
  public Point getIntakePosition() {
    return getIntakePosition(mPeriodicIO.pivotAngle.getDegrees());
  }

  public Point getIntakeTargetPosition() {
    return getIntakePosition(mPeriodicIO.pivotTarget);
  }

  private Point getIntakePosition(double pivotDeg) {
    double pivotDegAdj = 180 + pivotDeg - (14.192);
    
    double pivotRad = Math.toRadians(pivotDegAdj);
    double x = Math.cos(pivotRad) * 17.553;
    double y = Math.sin(pivotRad) * 17.553;

    // Adjust coordinates to the same as the shoulder
    x -= 20.125;
    y -= .54;

    return new Point(x, y);
  }

  public double calcPivotArbFF() {
    double angle = mPeriodicIO.pivotAngle.getDegrees();
    double cos = Math.cos(Math.toRadians(Math.abs(angle)));
    return cos * Constants.Intake.kPivotMaxArbFF * -1.0;
  }

  public double calcPivotPosFromAngle(double angle) {
    return ((Math.toRadians(angle)) / kPivotPositionCoefficient);
  }

  public void setPivotOutput(double output) {
    mPeriodicIO.pivotTarget = output;
  }

  public void setPivotVelocity(double velocity) {
    mPeriodicIO.pivotTarget = velocity;
  }

  private void writePivotOutput(double output) {
    pivotMotor.set(ControlMode.PercentOutput, output);
  }

  private void writePivotVelocity(double vel) {
    pivotMotor.set(ControlMode.Velocity, vel * Constants.Intake.kPivotCruiseVel, DemandType.ArbitraryFeedForward,
        calcPivotArbFF());
  }

  private void writePivotAngle(double angle) {
    double finalAngle = angle;

    Point intakePos = getIntakePosition();
    Point intakeTargetPos = getIntakeTargetPosition();

    Point armPos = mArm.getArmPosition();
    Point armTargetPos = mArm.getArmTargetPosition();
    
    // The intake is currently up.
    //if(mPeriodicIO.pivotAngle.getDegrees() < PivotPosition.UP.mAngle - 10) {
    //  // If either joint is back we don't want to move.
    //  if(armPos.x < 1 || armTargetPos.x < 1) {
    //    finalAngle = PivotPosition.UP.mAngle;
    //  }
    //}
    // The intake is past vertical and the arm is close to vertical
    if(intakePos.x < -25 && armPos.x < -20 && armPos.y < 16) {
      finalAngle = PivotPosition.DOWN.mAngle;
    }
    // The intake is trying to go beyond vertical and the arm isn't safe
    else if(intakePos.x < -20 && intakeTargetPos.x > -25 && armPos.x < -3 && armPos.x > intakePos.x) {
      finalAngle = PivotPosition.HANDOFF_CONE.mAngle;
    }
    else if(intakePos.x > -9 && intakeTargetPos.x < -9 && armPos.x < -3 && armPos.x > intakePos.x) {
      finalAngle = PivotPosition.UP.mAngle;
    }

    pivotMotor.set(ControlMode.MotionMagic, calcPivotPosFromAngle(finalAngle), DemandType.ArbitraryFeedForward,
        calcPivotArbFF());
  }

  public Rotation2d getPivotAngle() {
    return Rotation2d.fromRadians(getUnclampedPivotAngleRadians());
  }

  public double getUnclampedPivotAngleRadians() {
    return (pivotMotor.getSelectedSensorPosition() * kPivotPositionCoefficient);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public synchronized void readPeriodicInputs() {
    mPeriodicIO.pivotAbs = Cancoders.getInstance().getIntakePivot().getAbsolutePosition();

    mPeriodicIO.pivotAngle = getPivotAngle();
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    writeIntake();
    writePivot();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void stop() {
    setIntake(IntakeMode.STOP);
    writeIntake();

    writePivotOutput(0);
  }

  @Override
  public boolean checkSystem() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void outputTelemetry() {
    if (Constants.Intake.kDebug) {
      SmartDashboard.putString("IP PosName", mPeriodicIO.pivotPosition.name());
      SmartDashboard.putNumber("IP Target", mPeriodicIO.pivotTarget);
      SmartDashboard.putNumber("IP Deg", mPeriodicIO.pivotAngle.getDegrees());
      SmartDashboard.putNumber("IP Abs", getPivotCanCoderAngle().getDegrees());
      //SmartDashboard.putNumber("IP Abs1", getAdjustedPivotCanCoderAngle());
      //SmartDashboard.putNumber("IP Calc Deg", Math.toDegrees(Math.toRadians(getAdjustedPivotCanCoderAngle() + 14) / kPivotAbsPositionCoefficient));
      //SmartDashboard.putNumber("IP Offset", Constants.Intake.kPivotOffset.getDegrees());
      SmartDashboard.putNumber("IP Arb FF", calcPivotArbFF());

      SmartDashboard.putNumber("I Amps", intakeMotor.getOutputCurrent());
      
      SmartDashboard.putNumber("I Vel", intakeMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("I Output", intakeMotor.getAppliedOutput());
      SmartDashboard.putNumber("I Temp", intakeMotor.getMotorTemperature());
      SmartDashboard.putNumber("IP Amps", pivotMotor.getStatorCurrent());

      Point pos = getIntakePosition();
      SmartDashboard.putString("IP Pos", String.format("X:%f, Y:%f", pos.x, pos.y));

      Point targetPos = getIntakeTargetPosition();
      SmartDashboard.putString("IP T-Pos", String.format("X:%f, Y:%f", targetPos.x, targetPos.y));
    }
  }
}
