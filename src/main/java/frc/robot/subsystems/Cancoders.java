package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.other.CanDeviceId;
import frc.lib.other.TalonUtil;
import frc.robot.Constants;

import java.util.Optional;

// Conatiner to hold the Cancoders so we can initialize them
// earlier than everything else and DI them to the swerve modules
public class Cancoders {
    private final CANCoder mFrontLeft;
    private final CANCoder mFrontRight;
    private final CANCoder mBackLeft;
    private final CANCoder mBackRight;

    private final CANCoder mArmShoulder;
    private final CANCoder mArmElbow;

    private final CANCoder mIntakePivot;

    private final CanTsObserver mFrontRightObserver;
    private final CanTsObserver mFrontLeftObserver;
    private final CanTsObserver mBackLeftObserver;
    private final CanTsObserver mBackRightObserver;

    private final CanTsObserver mArmShoulderObserver;
    private final CanTsObserver mArmElbowObserver;

    private final CanTsObserver mIntakePivotObserver;

    private static final double kBootUpErrorAllowanceTime = 10.0;

    private static class CanTsObserver {
        private final CANCoder cancoder;
        private Optional<Double> lastTs = Optional.empty();
        private int validUpdates = 0;
        private static final int kRequiredValidTimestamps = 10;

        public CanTsObserver(CANCoder cancoder) {
            this.cancoder = cancoder;
        }

        public boolean hasUpdate() {
            cancoder.getAbsolutePosition(); // Need to call this to update ts
            double ts = cancoder.getLastTimestamp();
            if (lastTs.isEmpty()) {
                lastTs = Optional.of(ts);
            }
            if (ts > lastTs.get()) {
                validUpdates++;
                lastTs = Optional.of(ts);
            }
            return validUpdates > kRequiredValidTimestamps;
        }

    }

    private static Cancoders sInstance;

    public static Cancoders getInstance() {
        if (sInstance == null) {
            sInstance = new Cancoders();
        }
        return sInstance;
    }

    private CANCoder build(CanDeviceId canDeviceId) {
        return build(canDeviceId, false); // Default to Counter-clockwise
    }

    private CANCoder build(CanDeviceId canDeviceId, boolean sensorDirection) {
        CANCoder thisCancoder = new CANCoder(canDeviceId.getDeviceNumber(), canDeviceId.getBus());
        thisCancoder.configFactoryDefault();
        CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
        canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfig.magnetOffsetDegrees = 0.0;
        canCoderConfig.sensorDirection = sensorDirection; // false = Counter-clockwise

        double startTime = Timer.getFPGATimestamp();
        boolean timedOut = false;
        boolean goodInit = false;
        int attempt = 1;
        while (!goodInit && !timedOut) {
            System.out.println("Initing CANCoder " + canDeviceId.getDeviceNumber() + " / attempt: " + attempt);
            ErrorCode settingsError = thisCancoder.configAllSettings(canCoderConfig, Constants.Can.kLongTimeoutMs);
            ErrorCode sensorDataError = thisCancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 50,
                    Constants.Can.kLongTimeoutMs);
            TalonUtil.checkError(settingsError, "Failed to configure CANCoder");
            TalonUtil.checkError(sensorDataError, "Failed to configure CANCoder update rate");

            goodInit = settingsError == ErrorCode.OK && sensorDataError == ErrorCode.OK;
            timedOut = (Timer.getFPGATimestamp()) - startTime >= kBootUpErrorAllowanceTime;
            attempt++;
        }

        return thisCancoder;
    }

    private Cancoders() {
        mFrontLeft = build(Constants.Drive.kFrontLeftEncoderId);
        mFrontLeftObserver = new CanTsObserver(mFrontLeft);

        mFrontRight = build(Constants.Drive.kFrontRightEncoderId);
        mFrontRightObserver = new CanTsObserver(mFrontRight);

        mBackLeft = build(Constants.Drive.kBackLeftEncoderId);
        mBackLeftObserver = new CanTsObserver(mBackLeft);

        mBackRight = build(Constants.Drive.kBackRightEncoderId);
        mBackRightObserver = new CanTsObserver(mBackRight);

        mArmElbow = build(Constants.Arm.kElbowEncoderId);
        mArmElbowObserver = new CanTsObserver(mArmElbow);

        mArmShoulder = build(Constants.Arm.kShoulderEncoderId, false);
        mArmShoulderObserver = new CanTsObserver(mArmShoulder);

        mIntakePivot = build(Constants.Intake.kPivotEncoderId, false);
        mIntakePivotObserver = new CanTsObserver(mIntakePivot);
    }

    public boolean allHaveBeenInitialized() {
        return mFrontLeftObserver.hasUpdate() && mFrontRightObserver.hasUpdate() && mBackLeftObserver.hasUpdate()
                && mBackRightObserver.hasUpdate() && mArmShoulderObserver.hasUpdate() && mArmElbowObserver.hasUpdate()
                && mIntakePivotObserver.hasUpdate();
    }

    public CANCoder getFrontLeft() {
        return mFrontLeft;
    }

    public CANCoder getFrontRight() {
        return mFrontRight;
    }

    public CANCoder getBackLeft() {
        return mBackLeft;
    }

    public CANCoder getBackRight() {
        return mBackRight;
    }

    public CANCoder getArmShoulder() {
        return mArmShoulder;
    }

    public CANCoder getArmElbow() {
        return mArmElbow;
    }

    public CANCoder getIntakePivot() {
        return mIntakePivot;
    }
}
