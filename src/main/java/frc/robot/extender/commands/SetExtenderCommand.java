package frc.robot.extender.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.extender.ExtenderConstants;
import frc.robot.extender.ExtenderSubsystem;

public final class SetExtenderCommand extends CommandBase {

    private final ExtenderSubsystem extenderSubsystem;
    private final double pos;

    public SetExtenderCommand(double pos, ExtenderSubsystem extenderSubsystem) {
        this.pos = pos;
        this.extenderSubsystem = extenderSubsystem;
        addRequirements(extenderSubsystem);
    }

    @Override
    public void execute() {
        if (extenderSubsystem.getDistance() < pos) {
            extenderSubsystem.extend();
        } else {
            extenderSubsystem.retract();
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(extenderSubsystem.getDistance() - pos) < ExtenderConstants.ERROR;
    }
    
    @Override
    public void end(boolean interrupted) {
        extenderSubsystem.stop();
    }
}
