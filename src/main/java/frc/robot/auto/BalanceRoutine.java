package frc.robot.auto;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.DriveSubsystem;

public final class BalanceRoutine extends CommandBase {
    
    private final DriveSubsystem driveSubsystem;
    private final boolean invertSpeeds;
    private final Timer delay = new Timer();
    private final MedianFilter filter = new MedianFilter(AutoConstants.BALANCE_FILTER_SIZE);
    private State state = State.WAIT_FOR_TILT_BACK;
    private double tbs, tfs/*, tcs*/;
    private double lastPitch;
    private long lastTime;

    public BalanceRoutine(DriveSubsystem driveSubsystem, boolean invertSpeeds) {
        this.driveSubsystem = driveSubsystem;
        this.invertSpeeds = invertSpeeds;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        tbs = (invertSpeeds ? -1 : 1) * AutoConstants.BALANCE_TILT_BACK_SPEED;
        tfs = (invertSpeeds ? -1 : 1) * AutoConstants.BALANCE_TILT_FORWARD_SPEED;
        // tcs = (invertSpeeds ? -1 : 1) * AutoConstants.BALANCE_CORRECTING_SPEED;
        lastTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double pitch = getPitch();
        long t = System.currentTimeMillis();
        double dPitch = filter.calculate((pitch - lastPitch) / (t - lastTime) * 1000);
        lastTime = t;
        switch (state) {
            case WAIT_FOR_TILT_BACK:
                driveSubsystem.set(new ChassisSpeeds(tbs, 0, 0));
                if (pitch < AutoConstants.BALANCE_TILT_BACK_THRESHOLD) {
                    state = state.next();
                    delay.reset();
                    delay.start();
                }
                break;
            case WAIT_FOR_HIGH_DPITCH:
                driveSubsystem.set(new ChassisSpeeds(tfs, 0, 0));
                if (dPitch >= AutoConstants.BALANCE_VELOCITY_THRESHOLD && delay.get() >= 1.5) {
                    state = state.next();
                    delay.restart();
                }
                break;
            case CORRECTING:
                driveSubsystem.set(new ChassisSpeeds(AutoConstants.BALANCE_CORRECTING_SPEED, 0, 0));
                if (delay.get() >= AutoConstants.BALANCE_CORRECTING_TIME) {
                    state = state.next();
                }
                break;
            default: // Just in case something funny happens, bail out
                state = State.DONE;
                break;
        }
        lastPitch = pitch;
        SmartDashboard.putString("BalanceState", state.name());
        SmartDashboard.putNumber("DPitch", dPitch);
    }

    private double getPitch() {
        return driveSubsystem.getPitch();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            driveSubsystem.enable1771BrickMode();
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE || Math.abs(driveSubsystem.getOdometryPose().getX()) > 5;
    }

    private enum State {
        WAIT_FOR_TILT_BACK,
        WAIT_FOR_HIGH_DPITCH,
        CORRECTING,
        DONE;

        private static final State[] VALUES = State.values();

        public State next() {
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
    }
}
