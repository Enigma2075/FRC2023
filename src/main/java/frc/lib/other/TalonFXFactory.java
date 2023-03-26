package frc.lib.other;

import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.ForwardLimitSourceValue;
import com.ctre.phoenixpro.signals.ForwardLimitTypeValue;
import com.ctre.phoenixpro.signals.ForwardLimitValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonFXFactory {

    private final static int kTimeoutMs = 100;

    public final static TalonFXConfiguration kDefaultConfiguration = new TalonFXConfiguration();

    static {
        kDefaultConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        kDefaultConfiguration.MotorOutput.DutyCycleNeutralDeadband = .04;
        kDefaultConfiguration.MotorOutput.PeakForwardDutyCycle = 1;
        kDefaultConfiguration.MotorOutput.PeakReverseDutyCycle = -1;
        
        // No equivalent for Sensor Initialization Strategy?
        kDefaultConfiguration.Feedback.FeedbackRotorOffset = 0;
        
        kDefaultConfiguration.CurrentLimits.StatorCurrentLimitEnable = false;
        kDefaultConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        kDefaultConfiguration.CurrentLimits.SupplyCurrentLimitEnable = false;
        kDefaultConfiguration.CurrentLimits.SupplyCurrentLimit = 20;
        kDefaultConfiguration.CurrentLimits.SupplyCurrentThreshold = 60;
        kDefaultConfiguration.CurrentLimits.SupplyTimeThreshold = .2;
        

        kDefaultConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        kDefaultConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        kDefaultConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        kDefaultConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        kDefaultConfiguration.HardwareLimitSwitch.ForwardLimitEnable = false;
        kDefaultConfiguration.HardwareLimitSwitch.ReverseLimitEnable = false;
        kDefaultConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
        kDefaultConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;

        kDefaultConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        kDefaultConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0;
        kDefaultConfiguration.OpenLoopRamps.TorqueOpenLoopRampPeriod = 0;
        kDefaultConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

        kDefaultConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        kDefaultConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0;
        kDefaultConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
    }

    public static class Config {
        public int CONTROL_FRAME_PERIOD_MS = 10;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        public int GENERAL_STATUS_FRAME_RATE_MS = 10;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

        // public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;
    }

    static {
        // This control frame value seems to need to be something reasonable to avoid the Talon's
        // LEDs behaving erratically. Potentially try to increase as much as possible.
        // kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 100;
        // kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        // kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        // kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        // kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        // kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        // kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
        // kSlaveConfiguration.ENABLE_SOFT_LIMIT = false;
    }

    // create a CANTalon with the default (out of the box) configuration
    public static TalonFX createDefaultTalon(CanDeviceId id) {
        return createTalon(id, kDefaultConfiguration);
    }

//     public static TalonFX createPermanentSlaveTalon(CanDeviceId slave_id, CanDeviceId master_id) {
//         if( slave_id.getBus() != master_id.getBus() ) {
//                 throw new RuntimeException("Master and Slave Talons must be on the same CAN bus");
//         }
//         final TalonFX talon = createTalon(slave_id, kSlaveConfiguration);
//         talon.set(ControlMode.Follower, master_id.getDeviceNumber());
//         return talon;
//     }

    public static TalonFX createTalon(CanDeviceId id, TalonFXConfiguration config) {
        TalonFX talon = new TalonFX(id.getDeviceNumber(), id.getBus());
        var stopRequest = new VoltageOut(0, true, false);
        talon.setControl(stopRequest);

        var configurator = talon.getConfigurator();
        configurator.apply(config);

        // No equivalent
        //talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        //talon.clearMotionProfileHasUnderrun(kTimeoutMs);
        //talon.clearMotionProfileTrajectories();

        talon.clearStickyFaults(kTimeoutMs);

        // No equivalent
        //talon.configNominalOutputForward(0, kTimeoutMs);
        //talon.configNominalOutputReverse(0, kTimeoutMs);
    
        // No equivalent
        //talon.configMotorCommutation(MotorCommutation.Trapezoidal);

        // No Equivalent
        //talon.setSensorPhase(config.SENSOR_PHASE);

        // No Equivalent
        //talon.selectProfileSlot(0, 0);

        // No Equivalent
        //talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs);
        //talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW,
        //        kTimeoutMs);

        // No Equivalent
        // talon.configVoltageCompSaturation(0.0, kTimeoutMs);
        // talon.configVoltageMeasurementFilter(32, kTimeoutMs);
        // talon.enableVoltageCompensation(false);

        //talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeoutMs);
        //talon.configIntegratedSensorInitializationStrategy(config.SENSOR_INITIALIZATION_STRATEGY, kTimeoutMs);
        //talon.configIntegratedSensorOffset(config.SENSOR_OFFSET_DEGREES, kTimeoutMs);

        //talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
        //        config.GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);
        //talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
        //        config.FEEDBACK_STATUS_FRAME_RATE_MS, kTimeoutMs);

        //talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
        //        config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, kTimeoutMs);
        //talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
        //        config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, kTimeoutMs);
        //talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
        //        config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, kTimeoutMs);

        //talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

        return talon;
    }
}