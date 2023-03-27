package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.claw.ClawSubsystem;
import frc.robot.elevator.ElevatorConstants;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.extender.ExtenderSubsystem;

public final class DumpRoutine extends CommandBase {

    private final ElevatorSubsystem elevatorSubsystem;
    private final ExtenderSubsystem extenderSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final Timer timer = new Timer();
    private State state = State.GRABBING;

    public DumpRoutine(ElevatorSubsystem elevatorSubsystem, ExtenderSubsystem extenderSubsystem, ClawSubsystem clawSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.extenderSubsystem = extenderSubsystem;
        this.clawSubsystem = clawSubsystem;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        State current = state;
        switch (state) {
            case GRABBING:
                clawSubsystem.set(true);
                if (timer.get() > 1) {
                    state = state.next();
                }
                break;
            case RAISING:
                clawSubsystem.set(true);
                elevatorSubsystem.setMotor(ElevatorConstants.MANUAL_UP_SPEED);
                if (elevatorSubsystem.getTopLimitSwitch()) {
                    state = state.next();
                }
                break;
            case EXTENDING:
                elevatorSubsystem.setMotor(ElevatorConstants.MANUAL_UP_SPEED);
                extenderSubsystem.extend();
                clawSubsystem.set(true);
                if (timer.get() > 2) {
                    state = state.next();
                }
                break;
            case DROPPING:
            elevatorSubsystem.setMotor(ElevatorConstants.MANUAL_UP_SPEED);
                extenderSubsystem.stop();
                clawSubsystem.set(false);
                if (timer.get() > 1) {
                    state = state.next();
                }
                break;
            case RETRACTING:
                elevatorSubsystem.setMotor(ElevatorConstants.MANUAL_UP_SPEED);
                extenderSubsystem.retract();
                if (timer.get() > 3) {
                    state = state.next();
                }
                break;
            case LOWERING:
                elevatorSubsystem.setMotor(ElevatorConstants.MANUAL_DOWN_SPEED);
                if (elevatorSubsystem.getBottomLimitSwitch()) {
                    state = state.next();
                }
                break;
            default:
                state = State.DONE;
                break;
        }
        if (current != state) {
            timer.reset();
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    private enum State {
        GRABBING,
        RAISING,
        EXTENDING,
        DROPPING,
        RETRACTING,
        LOWERING,
        DONE;
        private static final State[] VALUES = State.values();

        public State next() {
            return VALUES[Math.min(ordinal() + 1, VALUES.length - 1)];
        }
    }
    
}
