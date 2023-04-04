package frc.robot.routines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.commands.AlignToTXCommand;

public enum TagAlignments {
    //CONE_LEFT(0),
    //CONE_RIGHT(0),
    CUBE(17.008802, -15.587663);

    private double txL, txR;
    
    private TagAlignments(double txL, double txR) {
        this.txL = txL;
        this.txR = txR;
    }

    public Command toCommand(DriveSubsystem driveSubsystem) {
        return new AlignToTXCommand(txL, txR, driveSubsystem);
    }
}
