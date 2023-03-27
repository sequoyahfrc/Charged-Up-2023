package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controls;
import frc.robot.drive.DriveConstants;
import frc.robot.drive.DriveSubsystem;

public final class DriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    public DriveCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {

        if (!DriverStation.isTeleopEnabled()) {
            return;
        }

        if (Controls.getDriver1BrickMode()) {
            driveSubsystem.enable1771BrickMode();
            return;
        }

        double f = Controls.getDriver1Forward();
        double l = Controls.getDriver1Left();
        double r = Controls.getDriver1TurnCCW();

        f = Math.abs(f) < 0.1 ? 0 : f;
        l = Math.abs(l) < 0.1 ? 0 : l;
        r = Math.abs(r) < 0.1 ? 0 : r;

        double boost = Controls.getSpeedBoost() * DriveConstants.SPEED_BOOST;

        f *= DriveConstants.MAX_SPEED + boost;
        l *= DriveConstants.MAX_SPEED + boost;
        r *= DriveConstants.MAX_ROTATION_SPEED;

        if (Controls.getDriver1SlowMode()) {
            f *= 0.5;
            l *= 0.5;
            r *= 0.5;
        }

        ChassisSpeeds speeds = new ChassisSpeeds(f, l, r);
        driveSubsystem.set(speeds, DriveConstants.MAX_SPEED + boost);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
