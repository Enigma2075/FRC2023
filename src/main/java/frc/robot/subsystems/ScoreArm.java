// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ScoreArm extends SubsystemBase {
  private final TalonFX longArm = new TalonFX(7);
  private final TalonFX shortArm = new TalonFX(8);
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
        Cancoders.getInstance().getFrontLeft(),
        Constants.Drive.kFrontLeftSteerOffset);

    mModules[kShortArmIdx] = new ArmPart(
        Constants.Drive.kFrontRightDriveId,
        Cancoders.getInstance().getFrontRight(),
        Constants.Drive.kFrontRightSteerOffset);
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
