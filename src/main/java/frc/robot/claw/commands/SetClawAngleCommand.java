package frc.robot.claw.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.claw.ClawConstants;
import frc.robot.claw.ClawSubsystem;

public final class SetClawAngleCommand extends CommandBase {

    private final ClawSubsystem clawSubsystem;
    private final PIDController pid = new PIDController(ClawConstants.WRIST_P, 0, 0);

    public SetClawAngleCommand(double angle, ClawSubsystem clawSubsystem) {
        this.clawSubsystem = clawSubsystem;
        pid.setTolerance(ClawConstants.WRIST_ERROR);
        pid.setSetpoint(angle);
        addRequirements(clawSubsystem);
    }

    @Override
    public void execute() {
        clawSubsystem.setWrist(pid.calculate(clawSubsystem.getAngle()));
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint() && Math.abs(clawSubsystem.getWristVelocity()) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setWrist(0);
    }
}
