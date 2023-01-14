package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.geometry.Twist2d;
import frc.lib.other.Subsystem;
import frc.lib.swerve.ChassisSpeeds;
import frc.lib.swerve.SwerveModuleState;
import frc.lib.swerve.SwerveSetpoint;
import frc.lib.swerve.SwerveSetpointGenerator;
import frc.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import frc.robot.Constants;

public class Drive extends Subsystem {
    private final Pigeon2 mPigeon = new Pigeon2(Constants.Drive.kPigeonIMUId, Constants.Can.kCANivoreBusName);

    private final SwerveModule[] mModules;
    public static final int kFrontLeftModuleIdx = 0;
    public static final int kFrontRightModuleIdx = 1;
    public static final int kBackLeftModuleIdx = 2;
    public static final int kBackRightModuleIdx = 3;

    private SwerveSetpointGenerator mSetpointGenerator;

    private double mYawOffset;
    private double mRollOffset;

    //private final DriveMotionPlanner mMotionPlanner;
    //private boolean mOverrideTrajectory = false;
    private DriveControlState mDriveControlState = DriveControlState.VELOCITY_CONTROL;
    private KinematicLimits mKinematicLimits = Constants.Drive.kFastKinematicLimits;

    private SwerveDriveOdometry mOdometry;

    private static Drive mInstance = null;

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    private Drive() {
        mModules = new SwerveModule[4];

        mModules[kFrontLeftModuleIdx] = new SwerveModule(
            Constants.Drive.kFrontLeftDriveId,
            Constants.Drive.kFrontLeftSteerId,
            Cancoders.getInstance().getFrontLeft(),
            Constants.Drive.kFrontLeftSteerOffset);

        mModules[kFrontRightModuleIdx] = new SwerveModule(
            Constants.Drive.kFrontRightDriveId,
            Constants.Drive.kFrontRightSteerId,
            Cancoders.getInstance().getFrontRight(),
            Constants.Drive.kFrontRightSteerOffset);

        mModules[kBackLeftModuleIdx] = new SwerveModule(
            Constants.Drive.kBackLeftDriveId,
            Constants.Drive.kBackLeftSteerId,
            Cancoders.getInstance().getBackLeft(),
            Constants.Drive.kBackLeftSteerOffset);

        mModules[kBackRightModuleIdx] = new SwerveModule(
            Constants.Drive.kBackRightDriveId,
            Constants.Drive.kBackRightSteerId,
            Cancoders.getInstance().getBackRight(),
            Constants.Drive.kBackRightSteerOffset);

        mYawOffset = mPigeon.getYaw();
        mRollOffset = mPigeon.getRoll();
        readGyro();
        readModules();
        setSetpointFromMeasured();

        mOdometry = new SwerveDriveOdometry(Constants.Drive.kKinematics.asWpiSwerveDriveKinematics(), mPeriodicIO.heading.asWpiRotation2d(), getWpiModulePositions());

        mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 5);

        mSetpointGenerator = new SwerveSetpointGenerator(Constants.Drive.kKinematics);

        //mMotionPlanner = new DriveMotionPlanner();

        resetSteer();
    }

    public static class PeriodicIO {
        // input/measured
        ChassisSpeeds des_chassis_speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveModuleState[] measured_states = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };

        double[] moduleAbsSteerAngleDeg = new double[] {0, 0, 0, 0};

        Rotation2d heading = new Rotation2d();
        Rotation2d roll = new Rotation2d();

        // outputs
        SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[Constants.Drive.kKinematics.getNumModules()]);
        Translation2d translational_error = new Translation2d();
        Rotation2d heading_error = new Rotation2d();
        //TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
        //TimedState<Rotation2d> heading_setpoint = new TimedState<>(Rotation2d.identity());

        boolean want_orient = false;
    }

    private final PeriodicIO mPeriodicIO = new PeriodicIO();

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public synchronized void zeroGyroscope() {
        mYawOffset = mPigeon.getYaw();
        mRollOffset = mPigeon.getRoll();
        readGyro();
    }

    public synchronized void setHeading(Rotation2d heading) {
        mYawOffset += mPeriodicIO.heading.getDegrees() - heading.getDegrees();
    }

    public synchronized Rotation2d getFieldRelativeGyroscopeRotation() {
        return mPeriodicIO.heading;
    }

    protected synchronized void readGyro() {
        mPeriodicIO.heading = Rotation2d.fromDegrees(mPigeon.getYaw() - mYawOffset);
        mPeriodicIO.roll = Rotation2d.fromDegrees(mPigeon.getRoll() - mRollOffset);
    }

    public synchronized Rotation2d getRoll() {
        return mPeriodicIO.roll;
    }

    public synchronized void resetRoll() {
        mRollOffset = mPigeon.getRoll();
    }

    public synchronized SwerveModuleState[] getModuleStates() {
        return mPeriodicIO.measured_states;
    }

    public synchronized ChassisSpeeds getDesiredChassisSpeeds() {
        return mPeriodicIO.des_chassis_speeds;
    }

    public SwerveModuleState[] getDesiredModuleStates () {
        return mPeriodicIO.setpoint.mModuleStates;
    }

    public synchronized SwerveSetpoint getSetpoint () {
        return mPeriodicIO.setpoint;
    }

    public synchronized edu.wpi.first.math.geometry.Pose2d getWpiPose() {
        return mOdometry.getPoseMeters();
    }

    public synchronized edu.wpi.first.math.kinematics.SwerveModulePosition[] getWpiModulePositions() {
        edu.wpi.first.math.kinematics.SwerveModulePosition[] wpiPositions = new edu.wpi.first.math.kinematics.SwerveModulePosition[mPeriodicIO.measured_states.length];
        for(int i = 0; i < mPeriodicIO.measured_states.length; i++) {
            wpiPositions[i] = mPeriodicIO.measured_states[i].asWpiSwerveModulePosition();
        }
        return wpiPositions;
    }

    public synchronized void resetWpiPose(edu.wpi.first.math.geometry.Pose2d pose) {
        SmartDashboard.putString("pose", pose.toString());
        mOdometry.resetPosition(mPeriodicIO.heading.asWpiRotation2d(), getWpiModulePositions(), pose);
    }

    public synchronized void setWpiModuleStates(edu.wpi.first.math.kinematics.SwerveModuleState[] wpiSwerveModuleStates) {
        SwerveModuleState[] moduleStates = new SwerveModuleState[mPeriodicIO.setpoint.mModuleStates.length];
        for(int i = 0; i < mPeriodicIO.setpoint.mModuleStates.length; i++) {
            moduleStates[i] = new SwerveModuleState(wpiSwerveModuleStates[i]);
        }
        
        mPeriodicIO.setpoint.mModuleStates = moduleStates;
    }

    public synchronized void setVelocity(ChassisSpeeds chassisSpeeds) {
        mPeriodicIO.des_chassis_speeds = chassisSpeeds;
        mPeriodicIO.want_orient = false;
        if (Constants.Drive.kUseVelocity) {
            mDriveControlState = DriveControlState.VELOCITY_CONTROL;
        } else {
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
    }

    public synchronized void setWantOrient(boolean wantOrient) {
        mPeriodicIO.want_orient = wantOrient;
        if (!wantOrient) {
            for (int i = 0; i < mModules.length; i++) {
                mModules[i].stop();
            }
        }
    }

    public synchronized void orientModules(List<Rotation2d> orientations) {
        setWantOrient(true);
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.setpoint.mModuleStates[i] =
                    new SwerveModuleState(0.0, orientations.get(i));
        }
    }

    public synchronized void orientModules(Rotation2d orientation) {
        setWantOrient(true);
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.setpoint.mModuleStates[i] = new SwerveModuleState(0.0, orientation);
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        readGyro();
        readModules();
    }

    private void readModules() {
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.measured_states[i] = mModules[i].getState();
            mPeriodicIO.moduleAbsSteerAngleDeg[i] = mModules[i].getCanCoderAngle().getDegrees();
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        for (int i = 0; i < mModules.length; ++i) {
            if (!mPeriodicIO.want_orient) {
                if (mDriveControlState == DriveControlState.VELOCITY_CONTROL || mDriveControlState == DriveControlState.PATH_FOLLOWING) {
                    mModules[i].setWithVelocityShortestPath(mPeriodicIO.setpoint.mModuleStates[i].speedMetersPerSecond,
                            mPeriodicIO.setpoint.mModuleStates[i].angle);
                } else if (mDriveControlState == DriveControlState.OPEN_LOOP) {
                    mModules[i].setWithVoltageShortestPath(
                            mPeriodicIO.setpoint.mModuleStates[i].speedMetersPerSecond / Constants.Drive.kMaxVelocityMetersPerSecond,
                            mPeriodicIO.setpoint.mModuleStates[i].angle);
                }
            } else {
                mModules[i].setWithVoltageShortestPath(0.0, mPeriodicIO.setpoint.mModuleStates[i].angle);
            }
        }
    }

    public synchronized void setSetpointFromMeasured() {
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.setpoint.mModuleStates[i] = mPeriodicIO.measured_states[i];
        }
        mPeriodicIO.setpoint.mChassisSpeeds = Constants.Drive.kKinematics.toChasisSpeedWheelConstraints(mPeriodicIO.setpoint.mModuleStates);
    }

    // TODO: Auton
    // public synchronized Pose2d getAutonSetpoint() {
    //     Pose2d ret = new Pose2d(mPeriodicIO.path_setpoint.state().getTranslation(), mPeriodicIO.heading_setpoint.state());
    //     return ret;
    // }

    // Reconfigure periodically in case an error was thrown the first time
    public void reconfigureTalons() {
        for (SwerveModule module : mModules) {
            module.configureTalons();
        }
    }

    public enum DriveControlState {
        OPEN_LOOP,
        VELOCITY_CONTROL,
        PATH_FOLLOWING
    }

    //TODO: Removed as it is path following

    // public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory) {
    //     if (mMotionPlanner != null) {
    //         mOverrideTrajectory = false;
    //         mMotionPlanner.reset();
    //         mMotionPlanner.setTrajectory(trajectory);
    //         mDriveControlState = DriveControlState.PATH_FOLLOWING;
    //     }
    // }

    // public synchronized boolean isDoneWithTrajectory() {
    //     if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
    //         return false;
    //     }
    //     return mMotionPlanner.isDone() || mOverrideTrajectory;
    // }

    // public synchronized void overrideTrajectory(boolean value) {
    //     mOverrideTrajectory = value;
    // }

    // public synchronized void setKinematicLimits(KinematicLimits limits) {
    //     if (limits != mKinematicLimits) {
    //         mKinematicLimits = limits;
    //     }
    // }

    // private void updatePathFollower() {
    //     if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
    //         final double now = Timer.getFPGATimestamp();

    //         Pose2d pose_meters = RobotStateEstimator.getInstance().getEstimatedPose();
    //         Pose2d pose_inches = new Pose2d(
    //             Units.meters_to_inches(pose_meters.getTranslation().x()),
    //             Units.meters_to_inches(pose_meters.getTranslation().y()),
    //             pose_meters.getRotation());

    //         ChassisSpeeds output =  mMotionPlanner.update(now, pose_inches);
    //         if (output != null) {
    //             if (!mPeriodicIO.want_orient) {   // hacky way to force orient modules... todo make better
    //                 mPeriodicIO.des_chassis_speeds = output;
    //             }
    //         }

    //         mPeriodicIO.translational_error = mMotionPlanner.getTranslationalError();
    //         mPeriodicIO.heading_error = mMotionPlanner.getHeadingError();
    //         mPeriodicIO.path_setpoint = mMotionPlanner.getPathSetpoint();
    //         mPeriodicIO.heading_setpoint = mMotionPlanner.getHeadingSetpoint();
    //     } else {
    //         DriverStation.reportError("Drive is not in path following state", false);
    //     }
    // }

    public synchronized void resetSteer() {
        for (SwerveModule module : mModules) {
            module.rezeroSteeringMotor();
        }
        // Force a module read.
        readModules();
    }

    public void setControlState(DriveControlState state) {
        mDriveControlState = state;
    }

    private void updateDesiredStates() {
        if (mPeriodicIO.want_orient) return;
        // Set the des_states to account for robot traversing arc.
        Pose2d robot_pose_vel = new Pose2d(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.Robot.kSecondsPerPeriodic,
                mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.Robot.kSecondsPerPeriodic,
                Rotation2d.fromRadians(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.Robot.kSecondsPerPeriodic));
        Twist2d twist_vel = Pose2d.log(robot_pose_vel);
        ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
                twist_vel.dx / Constants.Robot.kSecondsPerPeriodic, twist_vel.dy / Constants.Robot.kSecondsPerPeriodic, twist_vel.dtheta / Constants.Robot.kSecondsPerPeriodic);
        mPeriodicIO.setpoint = mSetpointGenerator.generateSetpoint(mKinematicLimits, mPeriodicIO.setpoint, updated_chassis_speeds, Constants.Robot.kSecondsPerPeriodic);
    }

    @Override
    public void periodic() {
        switch (mDriveControlState) {
            case PATH_FOLLOWING:
                mOdometry.update(mPeriodicIO.heading.asWpiRotation2d(), getWpiModulePositions());
                //setKinematicLimits(Constants.kFastKinematicLimits);
                //updatePathFollower();
            break;
            case OPEN_LOOP:
            case VELOCITY_CONTROL:
                updateDesiredStates();
            default:
                break;
        }
        updateDesiredStates();
    }

    //TODO: OnStart();
    // @Override
    // public void registerEnabledLoops(ILooper in) {
    //     in.register(new Loop() {
    //         @Override
    //         public void onStart(double timestamp) {
    //             setKinematicLimits(Constants.kFastKinematicLimits);
    //             for (int i = 0; i < mModules.length; i++) {
    //                 mModules[i].setSteerBrakeMode();
    //             }
    //         }
    // }

    public synchronized double[] getSteerClosedLoopErrors() {
        double[] rv = new double[]{0, 0, 0, 0};
        for (int i = 0; i < mModules.length; ++i) {
            rv[i] = mModules[i].getSteerClosedLoopError();
        }
        return rv;
    }

    // TODO: Auto
    // public synchronized Pose2d getAutonError() {
    //     return new Pose2d(mMotionPlanner.getTranslationalError(), mMotionPlanner.getHeadingError());
    // }

    @Override
    public void stop() {
        setVelocity(new ChassisSpeeds());
        for (int i = 0; i < mModules.length; i++) {
            mModules[i].stop();
        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        edu.wpi.first.math.geometry.Pose2d pose = mOdometry.getPoseMeters();
        SmartDashboard.putString("Wpi Odometry", String.format("X:%f, Y:%f, Rot:%f", 
        pose.getX(), pose.getY(), pose.getRotation().getDegrees()));

        SmartDashboard.putString("FL State", mPeriodicIO.setpoint.mModuleStates[kFrontLeftModuleIdx].toString());

        SmartDashboard.putString("Chassis Speeds", mPeriodicIO.des_chassis_speeds.toString());
        SmartDashboard.putString("Gyro Rot", getFieldRelativeGyroscopeRotation().toString());
        //SmartDashboard.putString("Gyro Roll", getRoll().toString());

        SmartDashboard.putNumber("FL Abs Deg", mPeriodicIO.moduleAbsSteerAngleDeg[kFrontLeftModuleIdx]);
        SmartDashboard.putNumber("FR Abs Deg", mPeriodicIO.moduleAbsSteerAngleDeg[kFrontRightModuleIdx]);
        SmartDashboard.putNumber("BL Abs Deg", mPeriodicIO.moduleAbsSteerAngleDeg[kBackLeftModuleIdx]);
        SmartDashboard.putNumber("BR Abs Deg", mPeriodicIO.moduleAbsSteerAngleDeg[kBackRightModuleIdx]);

        SmartDashboard.putNumber("FL Deg", mPeriodicIO.measured_states[kFrontLeftModuleIdx].angle.getDegrees());
        SmartDashboard.putNumber("FR Deg", mPeriodicIO.measured_states[kFrontRightModuleIdx].angle.getDegrees());
        SmartDashboard.putNumber("BL Deg", mPeriodicIO.measured_states[kBackLeftModuleIdx].angle.getDegrees());
        SmartDashboard.putNumber("BR Deg", mPeriodicIO.measured_states[kBackRightModuleIdx].angle.getDegrees());

        SmartDashboard.putString("FL State", mPeriodicIO.setpoint.mModuleStates[kFrontLeftModuleIdx].toString());
        SmartDashboard.putString("FR State", mPeriodicIO.setpoint.mModuleStates[kFrontRightModuleIdx].toString());
        SmartDashboard.putString("BL State", mPeriodicIO.setpoint.mModuleStates[kBackLeftModuleIdx].toString());
        SmartDashboard.putString("BR State", mPeriodicIO.setpoint.mModuleStates[kBackRightModuleIdx].toString());
        
        //SmartDashboard.putString("MAC", Constants.getMACAddress());

        //SmartDashboard.putString("Odometry", RobotStateEstimator.getInstance().getEstimatedPose().toString());

//        SmartDashboard.putNumber("X Error", mPeriodicIO.translational_error.x());
//        SmartDashboard.putNumber("Y Error", mPeriodicIO.translational_error.y());

//        SmartDashboard.putString("Auton Path Setpoint", mPeriodicIO.path_setpoint.state().getTranslation().toString());
//        SmartDashboard.putString("Auton Heading Setpoint", mPeriodicIO.heading_setpoint.state().getRotation().toString());

//        double[] errs = getSteerClosedLoopErrors();
//        SmartDashboard.putNumber("Front Left Azi Closed Loop Error", Math.toDegrees(errs[kFrontLeftModuleIdx]));
//        SmartDashboard.putNumber("Front Right Azi Closed Loop Error", Math.toDegrees(errs[kFrontRightModuleIdx]));
//        SmartDashboard.putNumber("Back Left Azi Closed Loop Error", Math.toDegrees(errs[kBackLeftModuleIdx]));
//        SmartDashboard.putNumber("Back Right Azi Closed Loop Error", Math.toDegrees(errs[kBackRightModuleIdx]));

//        SmartDashboard.putNumber("Front Left Azi Angle", mModules[kFrontLeftModuleIdx].getSteerAngle().getDegrees());
//        SmartDashboard.putNumber("Front Right Azi Angle", mModules[kFrontRightModuleIdx].getSteerAngle().getDegrees());
//        SmartDashboard.putNumber("Back Left Azi Angle", mModules[kBackLeftModuleIdx].getSteerAngle().getDegrees());
//        SmartDashboard.putNumber("Back Right Azi Angle", mModules[kBackRightModuleIdx].getSteerAngle().getDegrees());
    }
}
