package frc.robot.drive.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.DriveConstants;
import frc.robot.drive.DriveSubsystem;

public final class TurnToCommand extends CommandBase {
    private final Rotation2d rotation;
    private final DriveSubsystem driveSubsystem;
    
    public TurnToCommand(Rotation2d rotation, DriveSubsystem driveSubsystem) {
        this.rotation = rotation;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double speed = Math.pow(getDelta() / 360.0 * 10.0, 2);
        if (Math.abs(speed) > DriveConstants.TURN_TO_MAX_SPEED) {
            speed = Math.signum(speed) * DriveConstants.TURN_TO_MAX_SPEED;
        }
        driveSubsystem.set(new ChassisSpeeds(0, 0, -speed));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getDelta()) <= DriveConstants.TURN_TO_ERROR;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    private double getDelta() {
        return rotation.getDegrees() - driveSubsystem.getGyro().getDegrees();
    }
}
