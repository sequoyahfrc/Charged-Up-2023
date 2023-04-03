package frc.robot.claw.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.claw.ClawConstants;
import frc.robot.claw.ClawSubsystem;

public final class ShootCommand extends CommandBase {

    private final Timer timer = new Timer();
    private final ClawSubsystem clawSubsystem;
    private final double speed;

    public ShootCommand(double speed, ClawSubsystem clawSubsystem) {
        this.speed = speed;
        this.clawSubsystem = clawSubsystem;

        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        clawSubsystem.setWrist(0);
        if (clawSubsystem.getWristVelocity() < 1) {
            timer.start();
            clawSubsystem.setIntake(speed);
        } else {
            clawSubsystem.setIntake(0);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() > ClawConstants.SHOOT_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        clawSubsystem.stop();
    }
    
}
