package frc.robot.drive.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.DriveSubsystem;

public final class GoToPoseCommandFactory {

    public static Command create(Pose2d pose, RotationStrategy strategy, DriveSubsystem driveSubsystem) {
        var driveTo = new DriveToCommand(pose.getTranslation(), driveSubsystem);
        var turnTo = new TurnToCommand(pose.getRotation(), driveSubsystem);
        switch (strategy) {
            case AFTER:
                return new SequentialCommandGroup(driveTo, turnTo);
            default:
            case BEFORE:
                return new SequentialCommandGroup(turnTo, driveTo);
            case DURING:
                return new ParallelCommandGroup(driveTo, turnTo);

        }
    }

    public enum RotationStrategy {
        BEFORE,
        AFTER,
        DURING
    }
}
