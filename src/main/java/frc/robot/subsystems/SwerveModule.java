package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.other.CanDeviceId;
import frc.lib.geometry.Rotation2d;
import frc.lib.other.Subsystem;
import frc.lib.other.TalonFXFactory;
import frc.lib.other.TalonUtil;
import frc.lib.swerve.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    private final TalonFX mSteeringMotor;
    private final TalonFX mDriveMotor;
    private final CANCoder mCanCoder;
    private final Rotation2d mEncoderZero;
    private Rotation2d mTalonOffset;

    private final double kDrivePositionCoefficient = Math.PI * Constants.Drive.kWheelDiameter * Constants.Drive.kDriveReduction / 2048.0;
    private final double kDriveVelocityCoefficient = kDrivePositionCoefficient * 10.0;

    private final double kSteerPositionCoefficient = 2.0 * Math.PI / 2048.0 * Constants.Drive.kSteerReduction;

    public SwerveModule(CanDeviceId driveId, CanDeviceId steeringId,  CANCoder cancoder, Rotation2d encoderZero) {
        mDriveMotor = TalonFXFactory.createDefaultTalon(driveId);
        mSteeringMotor = TalonFXFactory.createDefaultTalon(steeringId);
        mCanCoder = cancoder;
        mEncoderZero = encoderZero;

        double start = Timer.getFPGATimestamp();
        while (Timer.getFPGATimestamp() - start < 10.0) {
            try {
                configureTalons();
                break;
            } catch (RuntimeException e) {
                Timer.delay(0.5);
                if (Timer.getFPGATimestamp() - start >= 10.0) {
                    System.out.println("TALON CONFIGURATION TIMED OUT: PORTS " + driveId + " AND " + steeringId);
                }
            }
        }

        rezeroSteeringMotor();

        stop();
    }

    public void configureTalons() throws RuntimeException {
        TalonUtil.checkErrorWithThrow(
                mDriveMotor.configVoltageCompSaturation(Constants.Drive.kMaxVoltage, Constants.Can.kLongTimeoutMs),
                "Failed to set voltage compensation");

        TalonUtil.checkErrorWithThrow(
                mDriveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 100.0, 120.0, 0.0), Constants.Can.kLongTimeoutMs),
                "Failed to set supply current limit");
        TalonUtil.checkErrorWithThrow(
                mDriveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 100.0, 120.0, 0.0), Constants.Can.kLongTimeoutMs),
                "Failed to set stator current limit");

        TalonUtil.checkErrorWithThrow(
                mDriveMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms, Constants.Can.kLongTimeoutMs),
                "Failed to set velocity measurement period");
        TalonUtil.checkErrorWithThrow(
                mDriveMotor.configVelocityMeasurementWindow(32, Constants.Can.kLongTimeoutMs),
                "Failed to set velocity measurement window");

        mDriveMotor.enableVoltageCompensation(true);
        mDriveMotor.setNeutralMode(NeutralMode.Brake);
        mDriveMotor.setInverted(TalonFXInvertType.Clockwise);
        mDriveMotor.setSensorPhase(true);
        mDriveMotor.setSelectedSensorPosition(0.0);
        mDriveMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms, 10);
        mDriveMotor.configVelocityMeasurementWindow(32, 10);

        // Reduce CAN status frame rates
        TalonUtil.checkErrorWithThrow(
                mDriveMotor.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_1_General,
                        250,
                        Constants.Can.kLongTimeoutMs
                ),
                "Failed to configure Falcon status frame period"
        );

        // Reduce CAN status frame rates
        TalonUtil.checkErrorWithThrow(
                mDriveMotor.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_2_Feedback0,
                        5,
                        Constants.Can.kLongTimeoutMs
                ),
                "Failed to configure Falcon status frame period"
        );
        mDriveMotor.setSelectedSensorPosition(0.0);

        // PID
        // TODO(get rid of dependency on Constants)
        TalonUtil.checkErrorWithThrow(
                mDriveMotor.config_kI(0, Constants.Drive.kVelocityKi, Constants.Can.kLongTimeoutMs),
                "Failed to set kI");
        TalonUtil.checkErrorWithThrow(
                mDriveMotor.config_kP(0, Constants.Drive.kVelocityKp, Constants.Can.kLongTimeoutMs),
                "Failed to set kP");
        TalonUtil.checkErrorWithThrow(
                mDriveMotor.config_kD(0, Constants.Drive.kVelocityKd, Constants.Can.kLongTimeoutMs),
                "Failed to set kD");
        TalonUtil.checkErrorWithThrow(
                mDriveMotor.config_kF(0, Constants.Drive.kVelocityKf, Constants.Can.kLongTimeoutMs),
                "Failed to set kF");

        // Steering
        TalonUtil.checkErrorWithThrow(
                mSteeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.Can.kLongTimeoutMs),
                "Failed to set encoder");
        TalonUtil.checkErrorWithThrow(
                mSteeringMotor.configVoltageCompSaturation(Constants.Drive.kMaxVoltage, Constants.Can.kLongTimeoutMs),
                "Failed to set voltage compensation");
        TalonUtil.checkErrorWithThrow(
                mSteeringMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 100.0, 120.0, 0.0), Constants.Can.kLongTimeoutMs),
                "Failed to set supply current limit");
        TalonUtil.checkErrorWithThrow(
                mSteeringMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 100.0, 120.0, 0.0), Constants.Can.kLongTimeoutMs),
                "Failed to set stator current limit");

        mSteeringMotor.enableVoltageCompensation(true);
        mSteeringMotor.setNeutralMode(NeutralMode.Coast);
        mSteeringMotor.setInverted(TalonFXInvertType.Clockwise);
        mSteeringMotor.setSensorPhase(false);

        // Reduce CAN status frame rates
        TalonUtil.checkErrorWithThrow(
                mSteeringMotor.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_1_General,
                        5,
                        Constants.Can.kLongTimeoutMs
                ),
                "Failed to configure Falcon status frame period"
        );

        // Reduce CAN status frame rates
        TalonUtil.checkErrorWithThrow(
                mSteeringMotor.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_2_Feedback0,
                        5,
                        Constants.Can.kLongTimeoutMs
                ),
                "Failed to configure Falcon status frame period"
        );

        // PID
        // TODO(get rid of dependency on Constants)
        TalonUtil.checkErrorWithThrow(
                mSteeringMotor.config_kP(0, Constants.Drive.kSteerKp, Constants.Can.kLongTimeoutMs),
                "Failed to set kP");
        TalonUtil.checkErrorWithThrow(
                mSteeringMotor.config_kI(0, Constants.Drive.kSteerKi, Constants.Can.kLongTimeoutMs),
                "Failed to set kI");
        TalonUtil.checkErrorWithThrow(
                mSteeringMotor.config_kD(0, Constants.Drive.kSteerKd, Constants.Can.kLongTimeoutMs),
                "Failed to set kD");
        TalonUtil.checkErrorWithThrow(
                mSteeringMotor.config_kF(0, 0.0, Constants.Can.kLongTimeoutMs),
                "Failed to set kF");
    }


    public void setSteerCoastMode() {
        mSteeringMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setSteerBrakeMode() {
        mSteeringMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void rezeroSteeringMotor() {
        mTalonOffset = Rotation2d.fromRadians(mSteeringMotor.getSelectedSensorPosition() * kSteerPositionCoefficient)
                .rotateBy(getAdjustedCanCoderAngle().inverse());
    }

    public Rotation2d getCanCoderAngle() {
        return Rotation2d.fromDegrees(mCanCoder.getAbsolutePosition());
    }

    public Rotation2d getAdjustedCanCoderAngle() {
        return getCanCoderAngle().rotateBy(mEncoderZero.inverse());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getDriveDistance(), getSteerAngle());
    }

    public double getDriveClosedLoopError() {
        return mDriveMotor.getClosedLoopError() * kDriveVelocityCoefficient;
    }

    public double getSteerClosedLoopError() { return mSteeringMotor.getClosedLoopError() * kSteerPositionCoefficient;}

    public double getDriveDistance() {
        return mDriveMotor.getSelectedSensorPosition() * kDrivePositionCoefficient;
    }

    public double getDriveVelocity() {
        return mDriveMotor.getSelectedSensorVelocity() * kDriveVelocityCoefficient;
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(getUnclampedSteerAngleRadians());
    }

    public double getUnclampedSteerAngleRadians() {
        return (mSteeringMotor.getSelectedSensorPosition() * kSteerPositionCoefficient) - mTalonOffset.getRadians();
    }

    public void setWithVoltageShortestPath(double drivePercentage, Rotation2d steerAngle) {
        final boolean flip = setSteerAngleShortestPath(steerAngle);
        mDriveMotor.set(TalonFXControlMode.PercentOutput, flip ? -drivePercentage : drivePercentage);
    }

    public void setWithVelocityShortestPath(double driveVelocity, Rotation2d steerAngle) {
        final boolean flip = setSteerAngleShortestPath(steerAngle);
        mDriveMotor.set(TalonFXControlMode.Velocity, (flip ? -driveVelocity : driveVelocity) / kDriveVelocityCoefficient);
    }

    public void setWithVoltageUnclamped(double drivePercentage, double steerAngleRadians) {
        setSteerAngleUnclamped(steerAngleRadians);
        mDriveMotor.set(TalonFXControlMode.PercentOutput, drivePercentage);
    }

    public void setWithVelocityUnclamped(double driveVelocity, double steerAngleRadians) {
        setSteerAngleUnclamped(steerAngleRadians);
        mDriveMotor.set(TalonFXControlMode.Velocity, driveVelocity / kDriveVelocityCoefficient);
    }

    // Returns true if the drive velocity should be inverted.
    private boolean setSteerAngleShortestPath(Rotation2d steerAngle) {
        boolean flip = false;
        final double unclampedPosition = getUnclampedSteerAngleRadians();
        final Rotation2d clampedPosition = Rotation2d.fromRadians(unclampedPosition);
        final Rotation2d relativeRotation = steerAngle.rotateBy(clampedPosition.inverse());
        double relativeRadians = relativeRotation.getRadians();
        final double kPiOver2 = Math.PI / 2.0;
        if (relativeRadians > kPiOver2) {
            // Flipping drive direction would be the shorter path.
            flip = true;
            relativeRadians -= Math.PI;
        } else if (relativeRadians < -kPiOver2) {
            // Flipping drive direction would be the shorter path.
            flip = true;
            relativeRadians += Math.PI;
        }
        setSteerAngleUnclamped(unclampedPosition + relativeRadians);

        return flip;
    }

    private void setSteerAngleUnclamped(double steerAngleRadians) {
        //System.out.println((steerAngleRadians + mTalonOffset.getRadians()) / kSteerPositionCoefficient);
        mSteeringMotor.set(TalonFXControlMode.Position, (steerAngleRadians + mTalonOffset.getRadians()) / kSteerPositionCoefficient);
    }

    public void stop() {
        System.out.println("stopping");
        setSteerCoastMode();
        mDriveMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        mSteeringMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }
}