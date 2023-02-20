package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StickyFaults;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.geometry.Rotation2d;
import frc.lib.other.Subsystem;
import frc.robot.Constants;

public class Intake extends Subsystem {// swhere you make it
  public enum Mode {
    CONE,
    CUBE
  }
  
  public enum PivotPosition {
    HANDOFF_CONE(-61.5),
    DOWN(-0),
    HANDOFF(-80),
    UP(-117);

    public final double mAngle;

    private PivotPosition(double angle) {
      this.mAngle = angle;
    }
  }

  public enum IntakeMode {
    CONE_IN(.9),
    CONE_OUT(-.9),
    CUBE_IN(-.9),
    CUBE_OUT(.9),
    CUBE_HOLD(-.02),
    STOP(0);

    public final double mOutput;

    private IntakeMode(double output) {
      this.mOutput = output;
    }
  }

  private final CANSparkMax intakeMotor;
  private final TalonSRX pivotMotor;

  private final double kPivotPositionCoefficient = 2.0 * Math.PI / 4096 * Constants.Intake.kPivotReduction;

  private double mPivotOffset;

  private static boolean mPivotReset = false;

  public static class PeriodicIO {
    Mode mode = Mode.CONE;

    // Pivot
    PivotPosition pivotPosition = PivotPosition.UP;
    double pivotTarget;
    Rotation2d pivotAngle;

    // Intake
    IntakeMode intakeMode = IntakeMode.STOP;
  }

  private final PeriodicIO mPeriodicIO = new PeriodicIO();

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    // Pivot Motor
    pivotMotor = new TalonSRX(Constants.Intake.kPivotId);

    pivotMotor.configFactoryDefault();

    pivotMotor.setNeutralMode(NeutralMode.Brake);

    pivotMotor.setInverted(InvertType.None);

    pivotMotor.configFeedbackNotContinuous(true, 10);

    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, 10);
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    pivotMotor.setSensorPhase(true);

    double pivotAbsPos = pivotMotor.getSelectedSensorPosition(1);
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

    pivotMotor.configNeutralDeadband(0);

    // Intake Motor
    intakeMotor = new CANSparkMax(Constants.Intake.kIntakeId, MotorType.kBrushless);

    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setIdleMode(IdleMode.kCoast);

    intakeMotor.setInverted(true);
  }

  public void setPivotPosition(PivotPosition position) {
    mPeriodicIO.pivotPosition = position;
  }

  public void setIntake(IntakeMode mode) {
    mPeriodicIO.intakeMode = mode;
  }

  public void updateIntake() {
    intakeMotor.set(mPeriodicIO.intakeMode.mOutput);
  }

  public void updatePivot() {
    if (mPivotReset) {
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
    } else {
      if (Math.abs(pivotMotor.getStatorCurrent()) > 40) {
        writePivotOutput(0);
        mPivotReset = true;
        double startingPosition = (Math.toRadians(-118 - 16) / kPivotPositionCoefficient);
        pivotMotor.setSelectedSensorPosition(startingPosition);
      } else {
        writePivotOutput(-1);
      }
    }
  }

  public CommandBase autoCommand(PivotPosition pivot, IntakeMode intake) {
    return runOnce(
        () -> {
          setPivotPosition(pivot);
          setIntake(intake);
        });
  }

  public CommandBase intakeCommand(Mode mode) {
    return runEnd(
        () -> {
          mPeriodicIO.mode = mode;

          setPivotPosition(PivotPosition.DOWN);
          switch(mPeriodicIO.mode) {
            case CUBE:
              setIntake(IntakeMode.CUBE_IN);
            break;
            case CONE:
              setIntake(IntakeMode.CONE_IN);
            break;
          }
        },
        () -> {
          switch(mPeriodicIO.mode) {
            case CUBE:
              setIntake(IntakeMode.CUBE_HOLD);
            break;
            case CONE:
              setIntake(IntakeMode.STOP);
            break;
          }
        });
  }

  public CommandBase outtakeCommand(Mode mode) {
    return runEnd(
        () -> {
          mPeriodicIO.mode = mode;
          switch(mPeriodicIO.mode) {
            case CUBE:
              setIntake(IntakeMode.CUBE_OUT);
            break;
            case CONE:
              setIntake(IntakeMode.CONE_OUT);
            break;
          }
        },
        () -> {
          setIntake(IntakeMode.STOP);
        });
  }

  public CommandBase defaultCommand(DoubleSupplier intakeSupplier, DoubleSupplier outakeSupplier) {
    return runEnd(
        () -> {
          // if (outakeSupplier.getAsDouble() > .8) {
          // setPivotOutput(1);
          // } else if (intakeSupplier.getAsDouble() > .8) {
          // setPivotOutput(-1);
          // } else {
          // setPivotOutput(0);
          // }

          if (intakeSupplier.getAsDouble() > .8) {
            mPeriodicIO.mode = Mode.CONE;
            setPivotPosition(PivotPosition.DOWN);
            setIntake(IntakeMode.CONE_IN);
          } else if (outakeSupplier.getAsDouble() > .8) {
            mPeriodicIO.mode = Mode.CONE;
            setIntake(IntakeMode.CONE_IN);
          } else {
            switch(mPeriodicIO.mode) {
              case CONE:
                setPivotPosition(PivotPosition.HANDOFF_CONE);
                setIntake(IntakeMode.STOP);
                break;
              case CUBE:
                setPivotPosition(PivotPosition.UP);
                setIntake(IntakeMode.CUBE_HOLD);
                break;
            }
          }
        },
        () -> {
        });
  }

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
    pivotMotor.set(ControlMode.MotionMagic, calcPivotPosFromAngle(angle), DemandType.ArbitraryFeedForward,
        calcPivotArbFF());
  }

  public Rotation2d getPivotAngle() {
    return Rotation2d.fromRadians(getUnclampedPivotAngleRadians());
  }

  public double getUnclampedPivotAngleRadians() {
    return ((pivotMotor.getSelectedSensorPosition()) * kPivotPositionCoefficient);
  }

  // public Rotation2d getPivotCanCoderAngle() {
  // return
  // Rotation2d.fromDegrees(Cancoders.getInstance().getArmShoulder().getAbsolutePosition());
  // }

  // public Rotation2d getAdjustedShoulderCanCoderAngle() {
  // return
  // getShoulderCanCoderAngle().rotateBy(Constants.Arm.kShoulderOffset.inverse());
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public synchronized void readPeriodicInputs() {
    mPeriodicIO.pivotAngle = getPivotAngle();
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    updateIntake();
    updatePivot();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void stop() {
    setIntake(IntakeMode.STOP);
    updateIntake();

    pivotMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public boolean checkSystem() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void outputTelemetry() {
    if (Constants.Intake.kDebug) {
      SmartDashboard.putNumber("IP Target", mPeriodicIO.pivotTarget);
      SmartDashboard.putNumber("IP Deg", mPeriodicIO.pivotAngle.getDegrees());
      double raw = pivotMotor.getSelectedSensorPosition(1);
      SmartDashboard.putNumber("IP Abs Raw", raw);
      SmartDashboard.putNumber("IP Arb FF", calcPivotArbFF());

      SmartDashboard.putNumber("I Vel", intakeMotor.getEncoder().getVelocity());
      SmartDashboard.putNumber("I Output", intakeMotor.getAppliedOutput());
      SmartDashboard.putNumber("IP Amps", pivotMotor.getStatorCurrent());
    }
  }
}
