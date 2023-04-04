package frc.robot.drive.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.DriveConstants;
import frc.robot.drive.DriveSubsystem;

public final class DriveToCommand extends CommandBase {
    private final Translation2d position;
    private final DriveSubsystem driveSubsystem;
    
    public DriveToCommand(Translation2d position, DriveSubsystem driveSubsystem) {
        this.position = position;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        var delta = getDelta();
        if (delta.getNorm() > DriveConstants.DRIVE_TO_MAX_SPEED) {
            delta = delta.times(DriveConstants.DRIVE_TO_MAX_SPEED / delta.getNorm());
        }
        driveSubsystem.set(new ChassisSpeeds(delta.getX(), delta.getY(), 0));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getDelta().getNorm()) <= DriveConstants.DRIVE_TO_ERROR;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    private Translation2d getDelta() {
        return position.minus(driveSubsystem.getOdometryPose().getTranslation());
    }
}
