package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.DriveSubsystem;

public final class AlignToTagCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final double targetX, targetZ;
    private final DoubleSubscriber tv, tv2, tx, tx2, ty, ty2;
    private final DoublePublisher pipelineL, pipelineR;
    private final DoubleArraySubscriber campose, campose2;
    private final boolean retro;
    private final Timer timer = new Timer();

    public AlignToTagCommand(double targetX, double targetZ, boolean retro, DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.targetX = targetX;
        this.targetZ = targetZ;
        this.retro = retro;

        addRequirements(driveSubsystem);

        NetworkTable tableL = NetworkTableInstance.getDefault().getTable("limelight-tags");
        NetworkTable tableR = NetworkTableInstance.getDefault().getTable("limelight-pieces");
        campose = tableL.getDoubleArrayTopic("campose").subscribe(new double[6]);
        campose2 = tableR.getDoubleArrayTopic("camerapose_targetspace").subscribe(new double[6]);
        tv = tableL.getDoubleTopic("tv").subscribe(0);
        tv2 = tableR.getDoubleTopic("tv").subscribe(0);
        tx = tableL.getDoubleTopic("tx").subscribe(0);
        tx2 = tableR.getDoubleTopic("tx").subscribe(0);
        ty = tableL.getDoubleTopic("ty").subscribe(0);
        ty2 = tableR.getDoubleTopic("ty").subscribe(0);

        pipelineL = tableL.getDoubleTopic("pipeline").publish();
        pipelineR = tableR.getDoubleTopic("pipeline").publish();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        pipelineL.set(retro ? 1 : 0);
        pipelineR.set(retro ? 1 : 0);
        if (timer.get() < 0.5) {
            return;
        }
        double errorX = targetX - getX();
        double errorZ = targetZ - getZ();
        double errorR = 180 - getAngle();
        double commandX = -Math.signum(errorX) * Math.abs(filter(4.0 * errorX));
        double commandZ = -Math.signum(errorZ) * Math.abs(filter(4.0 * errorZ));
        double commandR = -Math.signum(errorR) * Math.abs(filter(0.1 * errorR));
        if (Math.abs(errorX) > 2 || Math.abs(errorZ) > 2) {
            commandR = 0;
        }
        if (retro) {
            commandX *= -0.2 * (Math.abs(errorX) < 1 ? 0.1 : 1);
            commandZ *= 0.2;
        }
        ChassisSpeeds speeds = new ChassisSpeeds(commandZ, commandX, commandR);
        driveSubsystem.set(speeds, false);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() < 0.5) {
            return false;
        }
        final double ERROR = retro ? 0.75 : 0.01;
        if (tv.get() < 0.5 && tv2.get() < 0.5) {
            return true;
        }
        return Math.abs(targetX - getX()) < ERROR && Math.abs(targetZ - getZ()) < ERROR && Math.abs(180 - getAngle()) < 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
        driveSubsystem.enable1771BrickMode();
        tv.close();
        tv2.close();
        tx.close();
        tx2.close();
        ty.close();
        ty2.close();
        campose.close();
        campose2.close();
        pipelineL.set(0);
        pipelineR.set(0);
        pipelineL.close();
        pipelineR.close();
    }

    private static double filter(double x) {
        double s = Math.signum(x);
        x = x < 1 ? Math.pow(x, 2) : x;
        return s * Math.max(Math.min(x, 2.0), 0);
    }

    private double getX() {
        if (retro) {
            if (tv.get() > 0.5) {
                return tx.get();
            } else if (tv2.get() > 0.5) {
                return tx2.get();
            }
            return targetX;
        }
        double[] a;
        if (tv.get() > 0.5 && (a = campose.get()).length > 0) {
            return a[0];
        } else if (tv2.get() > 0.5 && (a = campose.get()).length > 0) {
            return a[0] + 0.348514;
        }
        return targetX;
    }
    
    private double getZ() {
        double[] a;
        if (retro) {
            if (tv.get() > 0.5) {
                return ty.get();
            } else if (tv2.get() > 0.5) {
                return ty2.get();
            }
            return targetZ;
        }
        if (tv.get() > 0.5 && (a = campose.get()).length >= 3) {
            return a[2];
        } else if (tv2.get() > 0.5 && (a = campose.get()).length >= 3) {
            return a[2] + 0.004296;
        }
        return targetZ;
    }

    private double getAngle() {
        double a = driveSubsystem.getGyro().getDegrees() % 360;
        return a;
    }
}
