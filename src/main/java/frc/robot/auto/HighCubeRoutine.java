package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.claw.ClawSubsystem;
import frc.robot.claw.commands.SetClawAngleCommand;
import frc.robot.claw.commands.ShootCommand;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.commands.SetElevatorCommand;

public final class HighCubeRoutine extends SequentialCommandGroup {

    public HighCubeRoutine(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
        addCommands(new SetElevatorCommand(20, elevatorSubsystem));
        addCommands(new WaitCommand(1));
        addCommands(new SetClawAngleCommand(65, clawSubsystem));
        addCommands(new WaitCommand(1));
        addCommands(new ShootCommand(-0.4, clawSubsystem));
        addCommands(new WaitCommand(1));
        addRequirements(elevatorSubsystem, clawSubsystem);
    }
    
}
