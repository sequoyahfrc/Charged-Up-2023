package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drive.DriveSubsystem;

public final class MobilityRoutine extends SequentialCommandGroup {

    public MobilityRoutine(DriveSubsystem driveSubsystem) {
        // addCommands(new RunCommand(() -> {
        //     driveSubsystem.set(new ChassisSpeeds(AutoConstants.MOBILITY_SPEED, 0, 0), false);
        // }, driveSubsystem).raceWith(new WaitCommand(AutoConstants.MOBILITY_TIME)));
        // addCommands(new InstantCommand(() -> {
        //     driveSubsystem.stop();
        // }, driveSubsystem));
        addCommands(new DriveForTimeCommand(new ChassisSpeeds(AutoConstants.MOBILITY_SPEED, 0, 0), AutoConstants.MOBILITY_TIME, driveSubsystem));
        addRequirements(driveSubsystem);
    }
    
}

