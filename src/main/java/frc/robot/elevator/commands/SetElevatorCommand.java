package frc.robot.elevator.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.ElevatorConstants;
import frc.robot.elevator.ElevatorSubsystem;

public final class SetElevatorCommand extends CommandBase {

    private final PIDController pid = new PIDController(ElevatorConstants.P, 0, ElevatorConstants.D);
    private final ElevatorSubsystem elevatorSubsystem;

    public SetElevatorCommand(double pos, ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
        pid.setSetpoint(pos);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setMotor(pid.calculate(elevatorSubsystem.getMotorPosition()));
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stop();
    }
}
