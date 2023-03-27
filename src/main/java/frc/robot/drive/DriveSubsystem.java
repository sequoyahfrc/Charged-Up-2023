package frc.robot.drive;

import static frc.robot.drive.DriveConstants.*;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LogFactory;
import frc.robot.drive.components.SwerveModule;

public final class DriveSubsystem extends SubsystemBase {

    private static final int FL_INDEX = 0;
    private static final int FR_INDEX = 1;
    private static final int BL_INDEX = 2;
    private static final int BR_INDEX = 3;

    private final SwerveModule frontLeft, frontRight, backLeft, backRight;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final AHRS gyro;
    private final SlewRateLimiter vxLimiter = new SlewRateLimiter(1 / DriveConstants.ACCELERATION_TIME, -1_000_000, 0);
    private final SlewRateLimiter vyLimiter = new SlewRateLimiter(1 / DriveConstants.ACCELERATION_TIME, -1_000_000, 0);
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1 / DriveConstants.ROTATION_ACCELERATION_TIME, -1_000_000, 0);
    private final Localizer localizer;
    private final DoubleArrayLogEntry currentLog = LogFactory.getDoubleArray("8080/Drive/Current");
    private final DoubleArrayLogEntry setpointsLog = LogFactory.getDoubleArray("8080/Drive/Setpoints");
    private double yawOffset, pitchOffset, rollOffset;

    public DriveSubsystem() {
        frontLeft = new SwerveModule("fl", AZIMUTH_FL_ID, DRIVE_FL_ID, ENCODER_FL_ID, ENCODER_FL_OFFSET);
        frontRight = new SwerveModule("fr", AZIMUTH_FR_ID, DRIVE_FR_ID, ENCODER_FR_ID, ENCODER_FR_OFFSET);
        backLeft = new SwerveModule("bl", AZIMUTH_BL_ID, DRIVE_BL_ID, ENCODER_BL_ID, ENCODER_BL_OFFSET);
        backRight = new SwerveModule("br", AZIMUTH_BR_ID, DRIVE_BR_ID, ENCODER_BR_ID, ENCODER_BR_OFFSET);

        Translation2d[] modulePos = new Translation2d[4];
        modulePos[FL_INDEX] = new Translation2d(LENGTH, -WIDTH);
        modulePos[FR_INDEX] = new Translation2d(LENGTH, WIDTH);
        modulePos[BL_INDEX] = new Translation2d(-LENGTH, -WIDTH);
        modulePos[BR_INDEX] = new Translation2d(-LENGTH, WIDTH);

        kinematics = new SwerveDriveKinematics(modulePos);
        gyro = new AHRS(Port.kMXP);
        gyro.zeroYaw();
        odometry = new SwerveDriveOdometry(kinematics, getGyro(), getPositions());
        localizer = new Localizer(this, odometry);

        yawOffset = pitchOffset = rollOffset = 0;

        SendableRegistry.add(this, "DriveSubsystem");
    }

    public Rotation2d getGyro() {
        return Rotation2d.fromDegrees(gyro.getAngle() - yawOffset);
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[FL_INDEX] = frontLeft.getPosition();
        positions[FR_INDEX] = frontRight.getPosition();
        positions[BL_INDEX] = backLeft.getPosition();
        positions[BR_INDEX] = backRight.getPosition();
        return positions;
    }

    public void set(ChassisSpeeds speeds) {
        set(speeds, MAX_SPEED);
    }

    public void set(ChassisSpeeds speeds, double maxSpeed) {
        set(speeds, true, maxSpeed);
    }

    public void set(ChassisSpeeds speeds, boolean fieldRelative) {
        set(speeds, fieldRelative, MAX_SPEED);
    }

    public void set(ChassisSpeeds speeds, boolean fieldRelative, double maxSpeed) {
        speeds.vxMetersPerSecond = Math.signum(speeds.vxMetersPerSecond) * vxLimiter.calculate(Math.abs(speeds.vxMetersPerSecond));
        speeds.vyMetersPerSecond = Math.signum(speeds.vyMetersPerSecond) * vyLimiter.calculate(Math.abs(speeds.vyMetersPerSecond));
        speeds.omegaRadiansPerSecond = Math.signum(speeds.omegaRadiansPerSecond) * omegaLimiter.calculate(Math.abs(speeds.omegaRadiansPerSecond));
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyro());
        }
        var states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);

        frontLeft.set(states[FL_INDEX]);
        frontRight.set(states[FR_INDEX]);
        backLeft.set(states[BL_INDEX]);
        backRight.set(states[BR_INDEX]);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // Turn wheels in on each other to stop moving
    // "borrowed" this idea from 1771
    public void enable1771BrickMode() {
        frontLeft.set(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        frontRight.set(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        backLeft.set(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        backRight.set(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    }

    @Override
    public void periodic() {
        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();

        odometry.update(getGyro(), getPositions());
        localizer.periodic();
        SmartDashboard.putData(gyro);
        SmartDashboard.putData("Modules/FL", frontLeft);
        SmartDashboard.putData("Modules/FR", frontRight);
        SmartDashboard.putData("Modules/BL", backLeft);
        SmartDashboard.putData("Modules/BR", backRight);
        SmartDashboard.putData(this);

        var current = new double[] {
            frontLeft.getAzimuth(), frontLeft.getVelocity(),
            frontRight.getAzimuth(), frontRight.getVelocity(),
            backLeft.getAzimuth(), backLeft.getVelocity(),
            backRight.getAzimuth(), backRight.getVelocity(),
        };
        currentLog.append(current);
        SmartDashboard.putNumberArray("DriveCurrent", current);
        var setpoints = new double[] {
            frontLeft.getAzimuthSetpoint(), frontLeft.getVelocitySetpoint(),
            frontRight.getAzimuthSetpoint(), frontRight.getVelocitySetpoint(),
            backLeft.getAzimuthSetpoint(), backLeft.getVelocitySetpoint(),
            backRight.getAzimuthSetpoint(), backRight.getVelocitySetpoint(),
        };
        setpointsLog.append(setpoints);
        SmartDashboard.putNumberArray("DriveSetpoints", setpoints);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("x", () -> getPose().getX(), d -> {});
        builder.addDoubleProperty("y", () -> getPose().getY(), d -> {});
        builder.addDoubleProperty("yaw", () -> getGyro().getDegrees(), d -> {});
        builder.addDoubleProperty("pitch", this::getPitch, d -> {});
        builder.addDoubleProperty("roll", this::getRoll, d -> {});
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
        //return localizer.getPose();
    }

    public double getRoll() {
        return gyro.getRoll() - rollOffset;
    }

    public double getPitch() {
        return gyro.getPitch() - pitchOffset;
    }

    public void calibrateGyro(Rotation2d realRotation) {
        gyro.setAngleAdjustment(realRotation.getDegrees() - getGyro().getDegrees());
    }

    public void reset() {
        frontLeft.reset();
        frontRight.reset();
        backLeft.reset();
        backRight.reset();
        yawOffset = pitchOffset = rollOffset = 0;
        yawOffset = getGyro().getDegrees();
        pitchOffset = getPitch();
        rollOffset = getRoll();
        odometry.resetPosition(getGyro(), getPositions(), new Pose2d(0, 0, getGyro()));
    }

    public void zero() {
        frontLeft.zero();
        frontRight.zero();
        backLeft.zero();
        backRight.zero();
    }

    public Optional<Integer> getPrimaryApriltag() {
        return localizer.getPrimaryApriltag();
    }
}
