package frc.robot.subsystems;

public abstract interface ArmMotionCondition {
    public abstract boolean conditionCheck(double shoulderAngle, double elbowAngle);
}


