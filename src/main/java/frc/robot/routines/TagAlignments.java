package frc.robot.routines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.commands.AlignToTagCommand;

public enum TagAlignments {
    CONE(0, 0, true), // Claibrated in LL dashboard
    CUBE(-0.20, -0.66, false);

    private final double x, z;
    private final boolean retro;
    
    private TagAlignments(double x, double z, boolean retro) {
        this.x = x;
        this.z = z;
        this.retro = retro;
    }

    public Command toCommand(DriveSubsystem driveSubsystem) {
        return new AlignToTagCommand(x, z, retro, driveSubsystem);
    }
}
