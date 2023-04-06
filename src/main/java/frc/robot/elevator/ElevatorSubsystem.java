package frc.robot.elevator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ElevatorSubsystem extends SubsystemBase {
    
    private final WPI_TalonFX left, right;
    private final DigitalInput bottomLimitSwitch, topLimitSwitch;
    private final SlewRateLimiter filter = new SlewRateLimiter(1 / ElevatorConstants.ACCELERATION_TIME);

    public ElevatorSubsystem() {
        left = new WPI_TalonFX(ElevatorConstants.LEFT_MOTOR_ID);
        right = new WPI_TalonFX(ElevatorConstants.RIGHT_MOTOR_ID);
        bottomLimitSwitch = new DigitalInput(ElevatorConstants.BOTTOM_LIMIT_SWITCH_ID);
        topLimitSwitch = new DigitalInput(ElevatorConstants.TOP_LIMIT_SWITCH_ID);

        left.configFactoryDefault();
        right.configFactoryDefault();
        left.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);
        right.setInverted(left.getInverted());
        right.follow(left);
        left.setSelectedSensorPosition(0);
        left.configPeakOutputForward(ElevatorConstants.MAX_SPEED);
        left.configPeakOutputReverse(-ElevatorConstants.MAX_SPEED);
        right.configPeakOutputForward(ElevatorConstants.MAX_SPEED);
        right.configPeakOutputReverse(-ElevatorConstants.MAX_SPEED);
        left.configVoltageCompSaturation(12);
        left.enableVoltageCompensation(true);
        right.configVoltageCompSaturation(12);
        right.enableVoltageCompensation(true);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("topSwitch", this::getTopLimitSwitch, b -> {});
        builder.addBooleanProperty("bottomSwitch", this::getBottomLimitSwitch, b -> {});
        builder.addDoubleProperty("motorPos", this::getMotorPosition, d -> {});
        builder.addDoubleProperty("motorPercent", left::get, d -> {});
    }

    @Override
    public void periodic() {
        if (getBottomLimitSwitch() && left.get() < 0) {
            left.stopMotor();
            left.setSelectedSensorPosition(0);
        }
        if (getTopLimitSwitch() && left.get() > 0) {
            left.set(0);
        }
        SmartDashboard.putData(this);
    }

    public void setMotor(double speed) {
        // Half speed if near top/bottom
        if (getMotorPosition() <= 5 && speed < 0) {
            speed *= 0.5;
        }
        if (getMotorPosition() >= 40 && speed > 0) {
            speed *= 0.5;
        }
        if (getBottomLimitSwitch() && speed < 0) {
            speed = 0;
        }
        if (getTopLimitSwitch() && speed > 0) {
            speed = 0;
        }
        left.set(filter.calculate(speed) + ElevatorConstants.KS);
    }

    public boolean getBottomLimitSwitch() {
        return !bottomLimitSwitch.get();
    }

    public boolean getTopLimitSwitch() {
        return !topLimitSwitch.get();
    }

    public double getMotorPosition() {
        return left.getSelectedSensorPosition() / 2048.0;
    }

    public void stop() {
        left.stopMotor();
    }
}
