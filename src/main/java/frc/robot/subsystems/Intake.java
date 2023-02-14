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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.geometry.Rotation2d;
import frc.lib.other.Subsystem;
import frc.robot.Constants;

public class Intake extends Subsystem {// swhere you make it
  public enum PivotPosition {
    DOWN(-10),
    UP(-130);
    
    public final double mAngle;

    private PivotPosition(double angle) {
      this.mAngle = angle;
    }
  }

  public enum IntakeMode {
    IN(.9),
    OUT(-.9),
    STOP(0);
    
    public final double mOutput;

    private IntakeMode(double output) {
      this.mOutput = output;
    }
  }

  private final CANSparkMax intakeMotor;
  private final TalonSRX pivotMotor;

  private final double kPivotPositionCoefficient = 2.0 * Math.PI / 4096 * Constants.Intake.kPivotReduction;

  public static class PeriodicIO {
    // Pivot
    PivotPosition pivotPosition = PivotPosition.UP;
    Rotation2d pivotAngle;


    // Intake
    IntakeMode intakeMode = IntakeMode.STOP;
  }

  private final PeriodicIO mPeriodicIO = new PeriodicIO();

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    // Pivot Motor
    pivotMotor = new TalonSRX(10);

    pivotMotor.configFactoryDefault();

    pivotMotor.setNeutralMode(NeutralMode.Brake);

    pivotMotor.setInverted(InvertType.InvertMotorOutput);

    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 1, 10);
    
    pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    pivotMotor.setSensorPhase(true);
    
    pivotMotor.setSelectedSensorPosition((Math.toRadians(-130 - Constants.Intake.kPivotOffset.getDegrees()) / kPivotPositionCoefficient));

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
    intakeMotor = new CANSparkMax(Constants.Intake.kIntakeId, MotorType.kBrushless);

    intakeMotor.restoreFactoryDefaults();

    intakeMotor.setIdleMode(IdleMode.kCoast);

    intakeMotor.setInverted(false);
  }

  public void setPivot(PivotPosition position) {
    mPeriodicIO.pivotPosition = position;
  }

  public void setIntake(IntakeMode mode) {
    mPeriodicIO.intakeMode = mode;
  }

  public void updateIntake() {
    intakeMotor.set(mPeriodicIO.intakeMode.mOutput);
  }

  public void updatePivot() {
    setPivotAngle(mPeriodicIO.pivotPosition.mAngle);
  }

  public CommandBase autoCommand(PivotPosition pivot, IntakeMode intake) {
    return runOnce(
      () -> {
        setPivot(pivot);
        setIntake(intake);
      }
    );
  }

  public CommandBase defaultCommand(DoubleSupplier intakeSupplier, DoubleSupplier outakeSupplier) {
    return runEnd(
        () -> {
          if (intakeSupplier.getAsDouble() > .8) {
            setPivot(PivotPosition.DOWN);
            setIntake(IntakeMode.IN);
          } else if (outakeSupplier.getAsDouble() > .8) {
            setIntake(IntakeMode.IN);
          } else {
            setPivot(PivotPosition.UP);
            setIntake(IntakeMode.STOP);
          }
          // setPivotOutput(outputSupplier.getAsDouble());
          // setPivotOutput(-.90);
        },
        () -> {
        });
  }

  public double calcPivotArbFF() {
    double angle = mPeriodicIO.pivotAngle.getDegrees();
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
    pivotMotor.set(ControlMode.Velocity, vel * Constants.Intake.kPivotCruiseVel, DemandType.ArbitraryFeedForward,
        calcPivotArbFF());
  }

  public void setPivotAngle(double angle) {
    pivotMotor.set(ControlMode.MotionMagic, calcPivotPosFromAngle(angle), DemandType.ArbitraryFeedForward,
        calcPivotArbFF());
  }

  public Rotation2d getPivotAngle() {
    return Rotation2d.fromRadians(getUnclampedPivotAngleRadians());
  }

  public double getUnclampedPivotAngleRadians() {
    return (pivotMotor.getSelectedSensorPosition() * kPivotPositionCoefficient);
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
    if(Constants.Intake.kDebug) {
      SmartDashboard.putNumber("IP Deg", mPeriodicIO.pivotAngle.getDegrees());
      SmartDashboard.putNumber("IP Abs", pivotMotor.getSelectedSensorPosition(1) * kPivotPositionCoefficient);
    }
  }
}
