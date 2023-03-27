package frc.robot.claw;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ClawSubsystem extends SubsystemBase {

    private final Solenoid solenoid;

    public ClawSubsystem() {
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, ClawConstants.SOLENOID_ID);
    }

    public void set(boolean value) {
        solenoid.set(value);
    }

    public boolean get() {
        return solenoid.get();
    }
}
