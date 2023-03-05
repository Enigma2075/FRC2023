package frc.robot.subsystems;

import org.opencv.core.Point;

public abstract interface ArmMotionCondition {
    public abstract boolean conditionCheck(Point p);
}


