package frc.robot.subsystems;

import org.opencv.core.Point;

import frc.robot.subsystems.Arm.ArmPosition;

public class ArmMotion {
    private ArmPosition mPosition;
    private ArmMotionCondition mElbowCondition;
    private ArmMotionCondition mShoulderCondition;

    public ArmMotion(ArmPosition position) {
      this(position, null, null);
    }

    public ArmMotion(ArmPosition position, ArmMotionCondition elbowCondition) {
      this(position, elbowCondition, null);
    }

    public ArmMotion(ArmPosition position, ArmMotionCondition elbowCondition, ArmMotionCondition shoulderCondition) {
      mPosition = position;
      mElbowCondition = elbowCondition;
      mShoulderCondition = shoulderCondition;
    }

    public ArmPosition getPosition() {
      return mPosition;
    }

    public boolean checkElbowCondition(double shoulderAngle, double elbowAngle) {
      if(mElbowCondition == null) {
       return true;
      }
      else {
        return mElbowCondition.conditionCheck(shoulderAngle, elbowAngle);
      }
    }

    public boolean checkShoulderCondition(double shoulderAngle, double elbowAngle) {
      if(mShoulderCondition == null) {
        return true;
      }
      else {
        return mShoulderCondition.conditionCheck(shoulderAngle, elbowAngle);
      }
    }
  }

