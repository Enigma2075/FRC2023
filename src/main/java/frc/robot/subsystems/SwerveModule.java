package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.CoastOut;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.StaticBrake;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

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
    private final TalonFXConfigurator mSteeringConfigurator;
    private final TalonFXConfiguration mSteeringConfiguration;
    private final TalonFX mDriveMotor;
    private final TalonFXConfigurator mDriveConfigurator;
    private final TalonFXConfiguration mDriveConfiguration;
    private final CANCoder mCanCoder;
    private final Rotation2d mEncoderZero;
    private Rotation2d mTalonOffset;

    private final double kDrivePositionCoefficient = Math.PI * Constants.Drive.kWheelDiameter * Constants.Drive.kDriveReduction / 1.0;
    private final double kDriveVelocityCoefficient = kDrivePositionCoefficient * 10.0;

    private final double kSteerPositionCoefficient = 2.0 * Math.PI / 1.0 * Constants.Drive.kSteerReduction;

    public SwerveModule(CanDeviceId driveId, CanDeviceId steeringId,  CANCoder cancoder, Rotation2d encoderZero) {
        mDriveMotor = TalonFXFactory.createDefaultTalon(driveId);
        mDriveConfigurator = mDriveMotor.getConfigurator();
        mDriveConfiguration = new TalonFXConfiguration();
        mDriveConfigurator.refresh(mDriveConfiguration);

        mSteeringMotor = TalonFXFactory.createDefaultTalon(steeringId);
        mSteeringConfigurator = mSteeringMotor.getConfigurator();
        mSteeringConfiguration = new TalonFXConfiguration();
        mSteeringConfigurator.refresh(mSteeringConfiguration);

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
        mDriveConfiguration.CurrentLimits.StatorCurrentLimit = 120;
        mDriveConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

        mDriveConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        mDriveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        mDriveConfiguration.Slot0.kP = Constants.Drive.kVelocityKp;
        mDriveConfiguration.Slot0.kI = Constants.Drive.kVelocityKi;
        mDriveConfiguration.Slot0.kD = Constants.Drive.kVelocityKd;
        mDriveConfiguration.Slot0.kV = Constants.Drive.kVelocityKf;
        //mDriveConfiguration.Slot0.kS = Constants.Drive.kVelocityKp;

        mDriveConfigurator.apply(mDriveConfiguration);

        // No Equivelant
        // TalonUtil.checkErrorWithThrow(
        //         mDriveMotor.configVoltageCompSaturation(Constants.Drive.kMaxVoltage, Constants.Can.kLongTimeoutMs),
        //         "Failed to set voltage compensation");

        // TalonUtil.checkErrorWithThrow(
        //         mDriveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 100.0, 120.0, 0.0), Constants.Can.kLongTimeoutMs),
        //         "Failed to set supply current limit");
        // TalonUtil.checkErrorWithThrow(
        //         mDriveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 100.0, 120.0, 0.0), Constants.Can.kLongTimeoutMs),
        //         "Failed to set stator current limit");

        // TalonUtil.checkErrorWithThrow(
        //         mDriveMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms, Constants.Can.kLongTimeoutMs),
        //         "Failed to set velocity measurement period");
        // TalonUtil.checkErrorWithThrow(
        //         mDriveMotor.configVelocityMeasurementWindow(32, Constants.Can.kLongTimeoutMs),
        //         "Failed to set velocity measurement window");


        //mDriveMotor.enableVoltageCompensation(true);
        //mDriveMotor.setSensorPhase(true);
        //mDriveMotor.setSelectedSensorPosition(0.0);
        
        //mDriveMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms, 10);
        //mDriveMotor.configVelocityMeasurementWindow(32, 10);

        // Reduce CAN status frame rates
        //TalonUtil.checkErrorWithThrow(
        //        mDriveMotor.setStatusFramePeriod(
        //                StatusFrameEnhanced.Status_1_General,
        //                250,
        //                Constants.Can.kLongTimeoutMs
        //        ),
        //        "Failed to configure Falcon status frame period"
        //);

        // Reduce CAN status frame rates
        //TalonUtil.checkErrorWithThrow(
        //        mDriveMotor.setStatusFramePeriod(
        //                StatusFrameEnhanced.Status_2_Feedback0,
        //                5,
        //                Constants.Can.kLongTimeoutMs
        //        ),
        //        "Failed to configure Falcon status frame period"
        //);
        //mDriveMotor.setSelectedSensorPosition(0.0);


        // Steering
        mSteeringConfiguration.CurrentLimits.StatorCurrentLimit = 100;
        mSteeringConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

        mSteeringConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        mSteeringConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        mSteeringConfiguration.Slot0.kP = Constants.Drive.kSteerKp;
        mSteeringConfiguration.Slot0.kI = Constants.Drive.kSteerKi;
        mSteeringConfiguration.Slot0.kD = Constants.Drive.kSteerKd;
        //mSteeringConfiguration.Slot0.kV = Constants.Drive.kSteerKf;
        //mSteeringConfiguration.Slot0.kS = Constants.Drive.kSteerKp;
        
        // No Equivelant
        // TalonUtil.checkErrorWithThrow(
        //         mSteeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.Can.kLongTimeoutMs),
        //         "Failed to set encoder");
        // TalonUtil.checkErrorWithThrow(
        //         mSteeringMotor.configVoltageCompSaturation(Constants.Drive.kMaxVoltage, Constants.Can.kLongTimeoutMs),
        //         "Failed to set voltage compensation");
        // TalonUtil.checkErrorWithThrow(
        //         mSteeringMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(false, 100.0, 120.0, 0.0), Constants.Can.kLongTimeoutMs),
        //         "Failed to set supply current limit");
        // TalonUtil.checkErrorWithThrow(
        //         mSteeringMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 100.0, 120.0, 0.0), Constants.Can.kLongTimeoutMs),
        //         "Failed to set stator current limit");

        //mSteeringMotor.enableVoltageCompensation(true);
        //mSteeringMotor.setNeutralMode(NeutralMode.Coast);
        //mSteeringMotor.setInverted(TalonFXInvertType.Clockwise);
        //mSteeringMotor.setSensorPhase(false);

        // Reduce CAN status frame rates
        // TalonUtil.checkErrorWithThrow(
        //         mSteeringMotor.setStatusFramePeriod(
        //                 StatusFrameEnhanced.Status_1_General,
        //                 5,
        //                 Constants.Can.kLongTimeoutMs
        //         ),
        //         "Failed to configure Falcon status frame period"
        // );

        // Reduce CAN status frame rates
        // TalonUtil.checkErrorWithThrow(
        //         mSteeringMotor.setStatusFramePeriod(
        //                 StatusFrameEnhanced.Status_2_Feedback0,
        //                 5,
        //                 Constants.Can.kLongTimeoutMs
        //         ),
        //         "Failed to configure Falcon status frame period"
        // );

        // PID
        // TODO(get rid of dependency on Constants)
        // TalonUtil.checkErrorWithThrow(
        //         mSteeringMotor.config_kP(0, Constants.Drive.kSteerKp, Constants.Can.kLongTimeoutMs),
        //         "Failed to set kP");
        // TalonUtil.checkErrorWithThrow(
        //         mSteeringMotor.config_kI(0, Constants.Drive.kSteerKi, Constants.Can.kLongTimeoutMs),
        //         "Failed to set kI");
        // TalonUtil.checkErrorWithThrow(
        //         mSteeringMotor.config_kD(0, Constants.Drive.kSteerKd, Constants.Can.kLongTimeoutMs),
        //         "Failed to set kD");
        // TalonUtil.checkErrorWithThrow(
        //         mSteeringMotor.config_kF(0, 0.0, Constants.Can.kLongTimeoutMs),
        //         "Failed to set kF");
    }


    public void setSteerCoastMode() {
       mSteeringMotor.setControl(new CoastOut());
    }

    public void setSteerBrakeMode() {
        mSteeringMotor.setControl(new StaticBrake());
    }

    public void rezeroSteeringMotor() {
        mTalonOffset = Rotation2d.fromRadians(mSteeringMotor.getRotorPosition().getValue() * kSteerPositionCoefficient)
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
        return mDriveMotor.getClosedLoopError().getValue() * kDriveVelocityCoefficient;
    }

    public double getSteerClosedLoopError() { return mSteeringMotor.getClosedLoopError().getValue() * kSteerPositionCoefficient;}

    public double getDriveDistance() {
        return mDriveMotor.getRotorPosition().getValue() * kDrivePositionCoefficient;
    }

    public double getDriveVelocity() {
        return mDriveMotor.getRotorVelocity().getValue() * kDriveVelocityCoefficient;
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromRadians(getUnclampedSteerAngleRadians());
    }

    public double getUnclampedSteerAngleRadians() {
        return (mSteeringMotor.getRotorPosition().getValue() * kSteerPositionCoefficient) - mTalonOffset.getRadians();
    }

    public void setWithVoltageShortestPath(double drivePercentage, Rotation2d steerAngle) {
        final boolean flip = setSteerAngleShortestPath(steerAngle);
        mDriveMotor.setControl(new DutyCycleOut(flip ? -drivePercentage : drivePercentage, true, false));
        //mDriveMotor.set(TalonFXControlMode.PercentOutput, flip ? -drivePercentage : drivePercentage);
    }

    public void setWithVelocityShortestPath(double driveVelocity, Rotation2d steerAngle) {
        final boolean flip = setSteerAngleShortestPath(steerAngle);
        mDriveMotor.setControl(new VelocityDutyCycle((flip ? -driveVelocity : driveVelocity) / kDriveVelocityCoefficient, true, 0, 0, false));
        //mDriveMotor.set(TalonFXControlMode.Velocity, (flip ? -driveVelocity : driveVelocity) / kDriveVelocityCoefficient);
    }

    public void setWithVoltageUnclamped(double drivePercentage, double steerAngleRadians) {
        setSteerAngleUnclamped(steerAngleRadians);
        mDriveMotor.setControl(new DutyCycleOut(drivePercentage, true, false));
        //mDriveMotor.set(TalonFXControlMode.PercentOutput, drivePercentage);
    }

    public void setWithVelocityUnclamped(double driveVelocity, double steerAngleRadians) {
        setSteerAngleUnclamped(steerAngleRadians);
        mDriveMotor.setControl(new VelocityDutyCycle(driveVelocity / kDriveVelocityCoefficient, true, 0, 0, false));
        //mDriveMotor.set(TalonFXControlMode.Velocity, driveVelocity / kDriveVelocityCoefficient);
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
        mSteeringMotor.setControl(new PositionDutyCycle((steerAngleRadians + mTalonOffset.getRadians()) / kSteerPositionCoefficient, true, 0, 0, false));
        //mSteeringMotor.set(TalonFXControlMode.Position, (steerAngleRadians + mTalonOffset.getRadians()) / kSteerPositionCoefficient);
    }

    public void stop() {
        System.out.println("stopping");
        setSteerCoastMode();
        mDriveMotor.setControl(new DutyCycleOut(0));
        mSteeringMotor.setControl(new DutyCycleOut(0));
        //mDriveMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        //mSteeringMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }
}