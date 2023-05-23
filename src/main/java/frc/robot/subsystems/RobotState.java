package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.other.Subsystem;

// Conatiner to hold the Cancoders so we can initialize them
// earlier than everything else and DI them to the swerve modules
public class RobotState extends Subsystem {
    public enum RobotMode {
        NORMAL, DEMO
    }

    public enum GamePieceMode {
        CONE, CUBE, UNKNOWN
    }

    private final double mBlinkOffDuration = .25;
    private final double mBlinkOnDuration = .25;

    private double mDriveSlowPercent = .5;

    private boolean mHasGamePiece;
    private double mBlinkTimer = 0;
    private boolean mLedsOn = true;

    private boolean mEnableVision = false;

    private final CANifier Leds;

    private GamePieceMode mGamePieceMode = GamePieceMode.CONE;

    private RobotMode mRobotMode = RobotMode.NORMAL;

    public RobotState() {
        Leds = new CANifier(1);

        setGamePeiceMode(GamePieceMode.CONE);
    }

    public boolean isCubeMode() {
        return mGamePieceMode == GamePieceMode.CUBE;
    }

    public boolean isConeMode() {
        return mGamePieceMode == GamePieceMode.CONE;
    }

    public boolean isVisionEnabled() {
        return mEnableVision;
    }

    public double getDriveSlowPercent() {
        return mDriveSlowPercent;
    }

    public void setVisionEnabled(boolean enableVision) {
        mEnableVision = enableVision;
    }

    public void setConeMode() {
        setConeMode(false);
    }
    
    public void setConeMode(boolean force) {
        setGamePeiceMode(GamePieceMode.CONE, force);
    }

    public void setCubeMode() {
        setCubeMode(false);
    }

    public void setDriveSlowPercent(double percent) {
        mDriveSlowPercent = percent;
    }

    public void setCubeMode(boolean force) {
        setGamePeiceMode(GamePieceMode.CUBE, force);
    }

    public void setHasGamePiece(boolean hasGamePiece) {
        mHasGamePiece = hasGamePiece;
    }

    public void setGamePeiceMode(GamePieceMode mode) {
        setGamePeiceMode(mode, false);
    }

    public void setGamePeiceMode(GamePieceMode mode, boolean force) {
        if (mHasGamePiece == false || force) {
            mGamePieceMode = mode;
        }
    }

    public void setRobotMode(RobotMode mode) {
        mRobotMode = mode;
    }

    public RobotMode getRobotMode() {
        return mRobotMode;
    }

    public boolean isDemoMode() {
        return mRobotMode == RobotMode.DEMO;
    }

    public boolean isNormalMode() {
        return mRobotMode == RobotMode.NORMAL;
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void writePeriodicOutputs() {
        if (mHasGamePiece) {
            if(Timer.getFPGATimestamp() - mBlinkTimer > mBlinkOffDuration && !mLedsOn) {
                mBlinkTimer = Timer.getFPGATimestamp();
                mLedsOn = !mLedsOn;
            }
            else if(Timer.getFPGATimestamp() - mBlinkTimer > mBlinkOnDuration && mLedsOn) {
                mBlinkTimer = Timer.getFPGATimestamp();
                mLedsOn = !mLedsOn;
            }
        }
        else if(!mHasGamePiece) {
            mLedsOn = true;
        }

        if (!mLedsOn) {
            Leds.setLEDOutput(0, CANifier.LEDChannel.LEDChannelA);
            Leds.setLEDOutput(0, CANifier.LEDChannel.LEDChannelB);
            Leds.setLEDOutput(0, CANifier.LEDChannel.LEDChannelC);
        } else {
            switch (mGamePieceMode) {
                case CUBE:
                    Leds.setLEDOutput(190.0 / 255.0, CANifier.LEDChannel.LEDChannelA);
                    Leds.setLEDOutput(0.0 / 255.0, CANifier.LEDChannel.LEDChannelB);
                    Leds.setLEDOutput(204.0 / 255.0, CANifier.LEDChannel.LEDChannelC);
                    break;
                case CONE:
                    Leds.setLEDOutput(255.0 / 255.0, CANifier.LEDChannel.LEDChannelA);
                    Leds.setLEDOutput(255.0 / 255.0, CANifier.LEDChannel.LEDChannelB);
                    Leds.setLEDOutput(0.0/ 255.0, CANifier.LEDChannel.LEDChannelC);
                    break;
            }
        }
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Game Piece Mode", mGamePieceMode.name());
        SmartDashboard.putString("Robot Mode", mRobotMode.name());
        SmartDashboard.putNumber("Slow Percent", mDriveSlowPercent);
    }
}
