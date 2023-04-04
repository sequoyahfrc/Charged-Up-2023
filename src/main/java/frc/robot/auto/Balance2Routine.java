package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.drive.DriveSubsystem;

public final class Balance2Routine extends SequentialCommandGroup {
    
    public Balance2Routine(DriveSubsystem driveSubsystem) {
        addCommands(new RunCommand(() -> {
            driveSubsystem.set(new ChassisSpeeds(AutoConstants.BALANCE2_MOBILITY_SPEED, 0, 0));
        }, driveSubsystem)
            .raceWith(new WaitUntilCommand(() -> Math.abs(driveSubsystem.getOdometryPose().getX()) > AutoConstants.BALANCE2_MOBILITY_X_THRESHOLD)));
        addCommands(new BalanceRoutine(driveSubsystem, true));
        addRequirements(driveSubsystem);
    }
}
