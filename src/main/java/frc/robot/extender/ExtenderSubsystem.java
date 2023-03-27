package frc.robot.extender;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ExtenderSubsystem extends SubsystemBase {

    private final Solenoid fwd, rev;
    private final I2C i2c;
    private final MedianFilter filter = new MedianFilter(ExtenderConstants.FILTER_SIZE);
    private double distance;

    public ExtenderSubsystem() {
        fwd = new Solenoid(PneumaticsModuleType.REVPH, ExtenderConstants.FORWARD_SOLENOID_ID);
        rev = new Solenoid(PneumaticsModuleType.REVPH, ExtenderConstants.REVERSE_SOLENOID_ID);
        i2c = new I2C(Port.kOnboard, 4);
        stop();
        SendableRegistry.add(this, "ExtenderSubsystem");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("ultrasonic", this::getDistance, d -> {});
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
        try {
            byte[] data = new byte[10];
            i2c.read(4, data.length, data);
            String s = new String(data);
            s = s.substring(0, s.indexOf('.'));
            distance = filter.calculate(Integer.parseInt(s));
        } catch (Throwable t) {
        }
    }

    public void extend() {
        fwd.set(true);
        rev.set(false);
    }

    public void retract() {
        fwd.set(false);
        rev.set(true);
    }

    public void stop() {
        fwd.set(true);
        rev.set(true);
    }
    
    public double getDistance() {
        return distance;
    }
}
