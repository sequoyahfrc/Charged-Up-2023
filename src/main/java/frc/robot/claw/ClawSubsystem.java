package frc.robot.claw;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public final class ClawSubsystem extends SubsystemBase {

    private final WPI_TalonFX intakeL, intakeR, wrist;
    //private final SlewRateLimiter wristLimiter = new SlewRateLimiter(ClawConstants.WRIST_ACCELERATION_TIME);

    public ClawSubsystem() {
        intakeL = new WPI_TalonFX(ClawConstants.INTAKE_L_ID);
        intakeR = new WPI_TalonFX(ClawConstants.INTAKE_R_ID);
        wrist = new WPI_TalonFX(ClawConstants.WRIST_ID);

        intakeL.configFactoryDefault();
        intakeR.configFactoryDefault();
        wrist.configFactoryDefault();
        wrist.configNeutralDeadband(0.01);
        wrist.configReverseSoftLimitEnable(true);
        wrist.configForwardSoftLimitEnable(true);
        wrist.configForwardSoftLimitThreshold(ClawConstants.CLAW_MAX_ANGLE / 360 * 2048 * ClawConstants.WRIST_GEAR_RATIO);
        wrist.configPeakOutputForward(ClawConstants.WRIST_MANUAL_UP_SPEED * 1.5);
        wrist.configPeakOutputReverse(ClawConstants.WRIST_MANUAL_DOWN_SPEED * 1.5);
        double clawDefault = -ClawConstants.HORIZONTAL_ANGLE / 360.0 * 2048.0 * ClawConstants.WRIST_GEAR_RATIO;
        wrist.setSelectedSensorPosition(clawDefault);
        wrist.configReverseSoftLimitThreshold(clawDefault);

        intakeL.configVoltageCompSaturation(12);
        intakeL.enableVoltageCompensation(true);
        intakeR.configVoltageCompSaturation(12);
        intakeR.enableVoltageCompensation(true);
        wrist.configVoltageCompSaturation(12);
        wrist.enableVoltageCompensation(true);

        intakeL.setNeutralMode(NeutralMode.Brake);
        intakeR.setNeutralMode(NeutralMode.Brake);
        
        wrist.setInverted(false);
        wrist.setNeutralMode(NeutralMode.Brake);
        stop();

        SendableRegistry.add(this, "ClawSubsystem");
    }

    public void setIntake(double speed) {
        speed = Math.max(Math.min(speed, 1), -1);
        intakeL.set(speed);
        // intakeR follows intakeL, do not set it here
    }

    public void setWrist(double speed) {
        speed = Math.max(Math.min(speed, 1), -1);
        // Add speed to hold at horizontal and account for angle of wrist (feed forward)
        double ff = ClawConstants.HORIZONTAL_KS * Math.cos(Math.toRadians(getAngle()));
        // TODO: if rate limit is needed, uncomment
        //wrist.set(wristLimiter.calculate(speed + ff));
        wrist.set(speed + ff);
    }

    public double getAngle() {
        return wrist.getSelectedSensorPosition() / ClawConstants.WRIST_GEAR_RATIO / 2048.0 * 360.0;
    }

    // This is in degrees/s
    public double getWristVelocity() {
        return wrist.getSelectedSensorVelocity() / ClawConstants.WRIST_GEAR_RATIO / 2048.0 * 360.0;
    }

    public void stop() {
        setIntake(0);
        setWrist(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("wristAngle", this::getAngle, x -> {});
        builder.addDoubleProperty("wristPercentOutput", wrist::get, x -> {});
        builder.addDoubleProperty("wristVelocity", this::getWristVelocity, x -> {});
        builder.addDoubleProperty("intakePercentOutput", intakeL::get, x -> {});
        builder.addDoubleProperty("intakeVelocity", () -> intakeL.getSelectedSensorVelocity() / 2048 / 4 * 360, x -> {});
    }

    @Override
    public void periodic() {
        intakeR.set(-intakeL.get());
        SmartDashboard.putData(this);
    }
}
