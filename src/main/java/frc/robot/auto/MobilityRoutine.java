package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.drive.DriveSubsystem;

public final class MobilityRoutine extends SequentialCommandGroup {

    public MobilityRoutine(DriveSubsystem driveSubsystem, boolean invert) {
        // addCommands(new RunCommand(() -> {
        //     driveSubsystem.set(new ChassisSpeeds(AutoConstants.MOBILITY_SPEED, 0, 0), false);
        // }, driveSubsystem).raceWith(new WaitCommand(AutoConstants.MOBILITY_TIME)));
        // addCommands(new InstantCommand(() -> {
        //     driveSubsystem.stop();
        // }, driveSubsystem));
        addCommands(new RunCommand(() -> {
            driveSubsystem.set(new ChassisSpeeds(AutoConstants.MOBILITY_SPEED, 0, 0), true);
        }, driveSubsystem)
            .raceWith(new WaitUntilCommand(() -> Math.abs(driveSubsystem.getPose().getX()) > 4  )));
        addCommands(new InstantCommand(() -> {
            driveSubsystem.stop();
        }));
        addRequirements(driveSubsystem);
    }
    
}

