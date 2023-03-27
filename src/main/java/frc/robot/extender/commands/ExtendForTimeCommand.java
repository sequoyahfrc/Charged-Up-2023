package frc.robot.extender.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.extender.ExtenderConstants;
import frc.robot.extender.ExtenderSubsystem;

public final class ExtendForTimeCommand extends CommandBase {

    private final ExtenderSubsystem extenderSubsystem;
    private final double time;
    private final Timer timer = new Timer();

    public ExtendForTimeCommand(double time, ExtenderSubsystem extenderSubsystem) {
        this.extenderSubsystem = extenderSubsystem;
        this.time = time;
        addRequirements(extenderSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        extenderSubsystem.extend();
        if (timer.hasElapsed(time)) {
            extenderSubsystem.retract();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time + ExtenderConstants.EXTENDER_COAST_COMPENSTAION_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        extenderSubsystem.stop();
    }
    
}
