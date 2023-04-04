package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.DriveSubsystem;

public final class AlignToTXCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final double targetL, targetR;
    private final DoubleSubscriber tx, tx2;
    private final BooleanSubscriber tv, tv2;

    public AlignToTXCommand(double targetL, double targetR, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.targetL = targetL;
        this.targetR = targetR;

        tx = NetworkTableInstance.getDefault().getTable("limelight-tags").getDoubleTopic("tx").subscribe(0);
        tx2 = NetworkTableInstance.getDefault().getTable("limelight-pieces").getDoubleTopic("tx").subscribe(0);
        tv = NetworkTableInstance.getDefault().getTable("limelight-tags").getBooleanTopic("tv").subscribe(false);
        tv2 = NetworkTableInstance.getDefault().getTable("limelight-pieces").getBooleanTopic("tv").subscribe(false);
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double command = 0;
        double error = 0;
        if (tv.get()) {
            error = targetL - tx.get(targetL);
        } else if (tv2.get()) {
            error = targetR - tx.get(targetR);
        }
        command = 0.1 * error;
        command = error < 1 ? Math.pow(command, 3) : command;
        command = Math.max(Math.min(command, 1), -1);
        ChassisSpeeds speeds = new ChassisSpeeds(0, command, 0);
        driveSubsystem.set(speeds, false);
    }

    @Override
    public boolean isFinished() {
        final double ERROR = 0.5;
        if (tv.get()) {
            return Math.abs(tx.get(targetL) - targetL) <= ERROR;
        }
        if (tv2.get()) {
            return Math.abs(tx2.get(targetR) - targetR) <= ERROR;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
        tx.close();
        tv.close();
    }
}
