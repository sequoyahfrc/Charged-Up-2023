package frc.robot.drive.components;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.drive.DriveConstants;

public final class SwerveModule implements Sendable {
    private final WPI_TalonFX azimuth, drive;
    private final PIDController pid;
    private final SimpleMotorFeedforward driveFF;
    private final AnalogInput encoder;
    private final double offset;
    //private final SlewRateLimiter azimuthLimiter = new SlewRateLimiter(1 / DriveConstants.MOTOR_ACCELERATION_TIME);
    //private final SlewRateLimiter driveLimiter = new SlewRateLimiter(1 / DriveConstants.MOTOR_ACCELERATION_TIME);

    private double azimuthSetpoint, driveSetpoint;

    public SwerveModule(String name, int azimuthID, int driveID, int encoder, double offset) {
        azimuth = new WPI_TalonFX(azimuthID);
        drive = new WPI_TalonFX(driveID);
        
        azimuth.configFactoryDefault();
        drive.configFactoryDefault();
        
        azimuth.setNeutralMode(NeutralMode.Coast);
        drive.setNeutralMode(NeutralMode.Coast);
        
        azimuth.configNeutralDeadband(0.01);
        drive.configNeutralDeadband(0.01);
        drive.setInverted(true);
        
        pid = new PIDController(DriveConstants.AZIMUTH_P, 0, DriveConstants.AZIMUTH_D);
        pid.enableContinuousInput(0, 360);
        pid.setTolerance(DriveConstants.AZIMUTH_ERROR, Double.MAX_VALUE);
        
        driveFF = new SimpleMotorFeedforward(DriveConstants.DRIVE_KS, 1.0);
        
        this.encoder = new AnalogInput(encoder);
        this.offset = offset;
        
        drive.setNeutralMode(NeutralMode.Coast);
        azimuth.setNeutralMode(NeutralMode.Coast);
        drive.setSelectedSensorPosition(0);
        azimuth.setSelectedSensorPosition(0);

        drive.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 80, 0.1));
        azimuth.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 80, 0.1));

        SendableRegistry.add(this, "SwerveModule-" + name);
    }

    public void set(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getAzimuth()));
        azimuthSetpoint = state.angle.getDegrees();
        driveSetpoint = state.speedMetersPerSecond;
    }

    public void zero() {
        calibrateIntegratedEncoder();
        driveSetpoint = 0;
        azimuthSetpoint = 0;
    }

    public void stop() {
        driveSetpoint = 0;
    }

    public void reset() {
        drive.setSelectedSensorPosition(0);
        zero();
    }

    public void calibrateIntegratedEncoder() {
        double absPosition = encoder.getValue() / 4096.0 * 5.0 / RobotController.getVoltage5V() * 360.0;
        absPosition = 360 - absPosition; // Absolute enoder is reversed
        azimuth.setSelectedSensorPosition((absPosition + offset) / 360.0 * DriveConstants.AZIMUTH_GEAR_RATIO * 2048.0);
    }

    public void periodic() {
        pid.setSetpoint(azimuthSetpoint);
        double aPID = pid.calculate(getAzimuth());
        if (!pid.atSetpoint()) {
            if (aPID < 0) {
                aPID = Math.min(aPID, DriveConstants.AZIMUTH_KS);
            } else {
                aPID = Math.max(aPID, DriveConstants.AZIMUTH_KS);
            }
            //aPID = azimuthLimiter.calculate(aPID);
            azimuth.setVoltage(aPID);
        } else {
            azimuth.set(0);
        }

        double dFF = driveFF.calculate(driveSetpoint);
        //dFF = driveLimiter.calculate(dFF);
        drive.setVoltage(dFF);
    }

    public double getAzimuth() {
        return azimuth.getSelectedSensorPosition() / 2048.0 / DriveConstants.AZIMUTH_GEAR_RATIO * 360.0;
    }

    public double getDriveDistance() {
        return drive.getSelectedSensorPosition() * DriveConstants.FALCON_500_ROTATIONS_TO_DISTANCE
                / DriveConstants.DRIVE_GEAR_RATIO;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromDegrees(getAzimuth()));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveModule");
        builder.addBooleanProperty("atSetpoint", pid::atSetpoint, b -> {});
        builder.addDoubleProperty("azimuth", this::getAzimuth, d -> {});
        builder.addDoubleProperty("azimuthSetpoint", () -> azimuthSetpoint, d -> {});
        builder.addDoubleProperty("drive", this::getVelocity, d -> {});
        builder.addDoubleProperty("driveDistance", this::getDriveDistance, d -> {});
        builder.addDoubleProperty("driveSetpoint", this::getVelocitySetpoint, d -> {});
    }

    public double getVelocity() {
        return drive.getSelectedSensorVelocity() * DriveConstants.FALCON_500_ROTATIONS_TO_DISTANCE / DriveConstants.DRIVE_GEAR_RATIO;
    }

    public double getVelocitySetpoint() {
        return driveSetpoint;
    }

    public double getAzimuthSetpoint() {
        return azimuthSetpoint;
    }
}
