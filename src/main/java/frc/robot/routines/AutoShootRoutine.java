package frc.robot.routines;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.claw.ClawSubsystem;
import frc.robot.claw.commands.SetClawAngleCommand;
import frc.robot.claw.commands.ShootCommand;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.commands.SetElevatorCommand;
import frc.robot.elevator.commands.ZeroElevatorCommand;

public enum AutoShootRoutine {
    MID_CUBE(10, 45, 0.2),
    HIGH_CUBE(20, 45, 0.4);

    private final double height, angle, speed;

    private AutoShootRoutine(double height, double angle, double speed) {
        this.height = height;
        this.angle = angle;
        this.speed = speed;
    }

    public Command toCommand(ElevatorSubsystem elevatorSubsystem, ClawSubsystem clawSubsystem) {
        return new SetElevatorCommand(height, elevatorSubsystem)
                .alongWith(new SetClawAngleCommand(angle, clawSubsystem))
            .andThen(new WaitCommand(1))
            .andThen(new ShootCommand(-speed, clawSubsystem))
            .andThen(new WaitCommand(0.1))
            .andThen(new SetClawAngleCommand(90, clawSubsystem)
                .alongWith(new ZeroElevatorCommand(elevatorSubsystem)));
    }
}
