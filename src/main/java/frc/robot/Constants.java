// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import edu.wpi.first.math.util.Units;
import frc.lib.other.CanDeviceId;
import frc.lib.swerve.SwerveDriveKinematics;
import frc.lib.swerve.SwerveSetpointGenerator.KinematicLimits;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriverStation {
    public static final int kDriverControllerPort = 0;

    public static final double kDriveJoystickThreshold = 0.1;
    public static final double kJoystickThreshold = 0.1;
  }

  public static class Robot {
    public static final String kPracticeBotMACAddress = "00:80:2F:33:D8:E9";
    public static final boolean kPracticeBot = getMACAddress().equals(kPracticeBotMACAddress);
    public static final double kSecondsPerPeriodic = .02;
  }

  public static class Can {
    public static final int kTimeoutMs = 10; // use for on the fly updates.
    public static final int kLongTimeoutMs = 100; // use in constructors.
  
    public static final String kRioBusName = "rio";
    public static final String kCANivoreBusName = "canivore";
  }

  public static class Drive {
    public static final double kCancoderBootAllowanceSeconds = 10.0;

    // If you want to just use voltage set this to false
    public static final boolean kUseVelocity = true;
    
    public static int kPigeonIMUId = 1;

    // TODO measure this
    public static final double kWheelDiameter = 0.10033 * 81.0 / 84.213; /// meters, TODO measure
    public static final double kTrackwidthMeters = 0.61595; // DONE Measure and set trackwidth
    public static final double kWheelbaseMeters = 0.61595; // DONE Measure and set wheelbase

    // Robot constants
    //public static final double kMaxDriveAcceleration = 1867 * 0.8;   // m/s^2 tuned 2/18 practice bot
    //public static final double kTrackScrubFactor = 1;

    //L3 Configuration
    //public static final double kDriveReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0); //TODO: need real values
    //public static final double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0); //TODO: need real values

    //L2 Configuration
    public static final double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); //TODO: need real values
    public static final double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0); //TODO: need real values

    
    // Measure the drivetrain's maximum velocity or calculate the theoretical.
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    // TODO measure this
    public static final double kMaxVelocityMetersPerSecond = 6380 / 60 * kDriveReduction * kWheelDiameter * Math.PI * .8;//4.959668;

    public static final double kMaxVoltage = 12.0;

    // Module Configurations
    public static final CanDeviceId kFrontLeftDriveId = new CanDeviceId(2, Can.kCANivoreBusName);
    public static final CanDeviceId kFrontLeftSteerId = new CanDeviceId(1, Can.kCANivoreBusName);
    public static final CanDeviceId kFrontLeftEncoderId = new CanDeviceId(1, Can.kCANivoreBusName);
    public static final Rotation2d kFrontLeftSteerOffset = Robot.kPracticeBot ? Rotation2d.fromDegrees(-49.482) : Rotation2d.fromDegrees(10.75);
    
    public static final CanDeviceId kFrontRightDriveId = new CanDeviceId(4, Can.kCANivoreBusName);
    public static final CanDeviceId kFrontRightSteerId = new CanDeviceId(3, Can.kCANivoreBusName);
    public static final CanDeviceId kFrontRightEncoderId = new CanDeviceId(2, Can.kCANivoreBusName);
    public static final Rotation2d kFrontRightSteerOffset = Robot.kPracticeBot ? Rotation2d.fromDegrees(-71.85) : Rotation2d.fromDegrees(67.5);

    public static final CanDeviceId kBackRightDriveId = new CanDeviceId(6, Can.kCANivoreBusName);
    public static final CanDeviceId kBackRightSteerId = new CanDeviceId(5, Can.kCANivoreBusName);
    public static final CanDeviceId kBackRightEncoderId = new CanDeviceId(3, Can.kCANivoreBusName);
    public static final Rotation2d kBackRightSteerOffset = Robot.kPracticeBot ? Rotation2d.fromDegrees(55.89) : Rotation2d.fromDegrees(201.5);

    public static final CanDeviceId kBackLeftDriveId = new CanDeviceId(8, Can.kCANivoreBusName);
    public static final CanDeviceId kBackLeftSteerId = new CanDeviceId(7, Can.kCANivoreBusName);
    public static final CanDeviceId kBackLeftEncoderId = new CanDeviceId(4, Can.kCANivoreBusName);
    public static final Rotation2d kBackLeftSteerOffset = Robot.kPracticeBot ? Rotation2d.fromDegrees(-40.86) : Rotation2d.fromDegrees(283.62);

    public static final double kSteerKp = 0.75;
    public static final double kSteerKi = 0;
    public static final double kSteerKd = 15;

    public static final double kVelocityKp = 0.1;
    public static final double kVelocityKi = 0.0;
    public static final double kVelocityKd = 0.01;
    public static final double kVelocityKf = 1023 / (kMaxVelocityMetersPerSecond / (Math.PI * kWheelDiameter * kDriveReduction / 2048.0 * 10));

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double kMaxAngularVelocityRadiansPerSecond = 11.386413;

    public static final double kScaleTranslationInputs = 1;
    public static final double kScaleRotationInputs = 0.5;

    public static final KinematicLimits kUncappedKinematicLimits = new KinematicLimits();
    static {
        kUncappedKinematicLimits.kMaxDriveVelocity = kMaxVelocityMetersPerSecond;
        kUncappedKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
        kUncappedKinematicLimits.kMaxSteeringVelocity = Double.MAX_VALUE;
    }

    public static final KinematicLimits kAzimuthOnlyKinematicLimits = new KinematicLimits();
    static {
        kAzimuthOnlyKinematicLimits.kMaxDriveVelocity = kMaxVelocityMetersPerSecond;
        kAzimuthOnlyKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
        kAzimuthOnlyKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1500.0);
    }

    public static final KinematicLimits kTeleopKinematicLimits = new KinematicLimits();
    static {
        kTeleopKinematicLimits.kMaxDriveVelocity = kMaxVelocityMetersPerSecond;
        kTeleopKinematicLimits.kMaxDriveAcceleration = kTeleopKinematicLimits.kMaxDriveVelocity / 0.1;
        kTeleopKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1500.0);
    }

    public static final KinematicLimits kFastKinematicLimits = new KinematicLimits();
    static {
        kFastKinematicLimits.kMaxDriveVelocity = kMaxVelocityMetersPerSecond;
        kFastKinematicLimits.kMaxDriveAcceleration = kFastKinematicLimits.kMaxDriveVelocity / 0.2;
        kFastKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(1000.0);
    }

    public static final KinematicLimits kSmoothKinematicLimits = new KinematicLimits();
    static {
        kSmoothKinematicLimits.kMaxDriveVelocity = kMaxVelocityMetersPerSecond * .7;
        kSmoothKinematicLimits.kMaxDriveAcceleration = kSmoothKinematicLimits.kMaxDriveVelocity / 1.0;
        kSmoothKinematicLimits.kMaxSteeringVelocity = Units.degreesToRadians(750.0);
    }

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0),
            // Front right
            new Translation2d(kTrackwidthMeters / 2.0, -kWheelbaseMeters / 2.0),
            // Back left
            new Translation2d(-kTrackwidthMeters / 2.0, kWheelbaseMeters / 2.0),
            // Back right
            new Translation2d(-kTrackwidthMeters / 2.0, -kWheelbaseMeters / 2.0)
    );
  }

  /**
   * @return the MAC address of the robot
   */
  public static String getMACAddress() {
    try {
        Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
        StringBuilder ret = new StringBuilder();
        while (nwInterface.hasMoreElements()) {
            NetworkInterface nis = nwInterface.nextElement();
            System.out.println("NIS: " + nis.getDisplayName());
            if (nis != null && "eth0".equals(nis.getDisplayName())) {
                byte[] mac = nis.getHardwareAddress();
                if (mac != null) {
                    for (int i = 0; i < mac.length; i++) {
                        ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                    }
                    String addr = ret.toString();
                    System.out.println("NIS " + nis.getDisplayName() + " addr: " + addr);
                    return addr;
                } else {
                    System.out.println("Address doesn't exist or is not accessible");
                }
            } else {
                System.out.println("Skipping adaptor: " + nis.getDisplayName());
            }
        }
    } catch (SocketException | NullPointerException e) {
        e.printStackTrace();
    }

    return "";
  }

}
