package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.DriveSubsystem;

public final class AlignToTXCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final double target;
    private final DoubleSubscriber tx;

    public AlignToTXCommand(double target, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.target = target;

        tx = NetworkTableInstance.getDefault().getTable("limelight-tags").getDoubleTopic("tx").subscribe(0);
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double tx = this.tx.get(target);
        double command = 0.1 * (target - tx);
        command = Math.max(Math.min(command, 1), -1);
        ChassisSpeeds speeds = new ChassisSpeeds(0, command, 0);
        driveSubsystem.set(speeds, false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(tx.get(target) - target) < 1;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.set(new ChassisSpeeds());
        tx.close();
    }
}
