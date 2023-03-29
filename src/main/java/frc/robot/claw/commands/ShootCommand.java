package frc.robot.claw.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.claw.ClawConstants;
import frc.robot.claw.ClawSubsystem;

public final class ShootCommand extends CommandBase {

    private final Timer timer = new Timer();
    private ClawSubsystem clawSubsystem;

    public ShootCommand(ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;

        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // TODO: start timer only after wrist stops moving (at zero velocity)
    @Override
    public void execute() {
        clawSubsystem.setWrist(0);
        if (timer.get() > ClawConstants.SHOOT_DELAY) {
            clawSubsystem.setIntake(ClawConstants.SHOOT_SPEED);
        } else {
            clawSubsystem.setIntake(0);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() > (ClawConstants.SHOOT_DELAY + ClawConstants.SHOOT_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        clawSubsystem.stop();
    }
    
}
