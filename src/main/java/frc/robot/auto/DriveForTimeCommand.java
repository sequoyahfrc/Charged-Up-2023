package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.drive.DriveSubsystem;

public final class DriveForTimeCommand extends RunCommand {

    private final Timer timer = new Timer();
    private final double time;
    private final DriveSubsystem driveSubsystem;

    public DriveForTimeCommand(ChassisSpeeds speed, double time, DriveSubsystem driveSubsystem) {
        super(() -> {
            driveSubsystem.set(speed, false);
        }, driveSubsystem);
        this.time = time;
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= time;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
    
}
