// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.geometry.Rotation2d;
import frc.robot.Constants;



public class ScoreArm extends SubsystemBase {
  private final CANSparkMax mStage2Motor;
  private final CANSparkMax mStage1RightMotor;
  private final CANSparkMax mStage1LeftMotor;

  private final RelativeEncoder mStage1Encoder;
  private final RelativeEncoder mStage2Encoder;

  private final double kStage1PositionCoefficient = 2.0 * Math.PI / 42.0 * Constants.Drive.kStage1Reduction;
  private final double kStage2PositionCoefficient = 2.0 * Math.PI / 42.0 * Constants.Drive.kStage2Reduction;

  public ScoreArm(){
    mStage2Motor = new CANSparkMax(Constants.Arm.kStage2Id, MotorType.kBrushless);
    mStage1LeftMotor = new CANSparkMax(Constants.Arm.kStage1RightId, MotorType.kBrushless);
    mStage1RightMotor = new CANSparkMax(Constants.Arm.kStage1LeftId, MotorType.kBrushless);
    
    mStage1Encoder = mStage1LeftMotor.getEncoder();
    mStage2Encoder = mStage2Motor.getEncoder();
  }

  
  public void rezeroSteeringMotor() {
    mNeoOffset = Rotation2d.fromRadians(mStage2Encoder.getPosition() * kStage2PositionCoefficient)
            .rotateBy(getAdjustedCanCoderAngle().inverse());
  }
//2 absolute cancoders
//gear ratios? convert to degrees somehow
//so basically, virtual 4 bar, need to find out how things move/relationship between? linear equation, long is x short is y? 
//if one goes one way, other goes other way, if x is +, y is - and vice versa. with gear ratio+encoder, find angles, use triangles?

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }


  /* 
private static ScoreArm mInstance = null;

private final SwerveModule[] mModules;

public static final int kLongArmIdx = 7;
public static final int kShortArmIdx = 8;

public static ScoreArm getInstance() {
  if (mInstance == null) {
      mInstance = new ScoreArm();
  }
  return mInstance;
}

private final ScoreArm[] mModules;

  private ScoreArm() {
    mModules = new ArmPart[2];

    mModules[kLongArmIdx] = new ArmPart(
        Constants.Drive.kFrontLeftDriveId,
        Cancoders.getInstance().getFrontLeft()
        );

    mModules[kShortArmIdx] = new ArmPart(
        Constants.Drive.kFrontRightDriveId,
        Cancoders.getInstance().getFrontRight()
        );
  }
*/


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
