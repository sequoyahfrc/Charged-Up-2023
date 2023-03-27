package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.elevator.ElevatorConstants;
import frc.robot.elevator.ElevatorSubsystem;

public final class ZeroElevatorCommand extends CommandBase {

    private final ElevatorSubsystem elevatorSubsystem;

    public ZeroElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setMotor(ElevatorConstants.ZERO_SPEED);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getBottomLimitSwitch();
    }
}
