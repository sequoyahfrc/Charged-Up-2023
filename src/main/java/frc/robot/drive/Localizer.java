package frc.robot.drive;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import frc.robot.LogFactory;

public final class Localizer {

    private static final Transform2d ZERO_TRANSFORM = new Transform2d(new Translation2d(), new Rotation2d());

    private final DriveSubsystem driveSubsystem;
    private final SwerveDriveOdometry odometry;
    private final BooleanSubscriber tv;
    private final IntegerSubscriber tid;
    private final DoubleArraySubscriber botpose;
    private final DoubleSubscriber tl;
    private final ArrayList<Pair<Pose2d, Double>> odomHistory = new ArrayList<>();
    private final DoubleArrayLogEntry apriltagPoseLog = LogFactory.getDoubleArray("8080/Localization/ApriltagPose");
    private final DoubleArrayLogEntry rawApriltagPoseLog = LogFactory.getDoubleArray("8080/Localization/UnfilteredApriltagPose");
    private final DoubleArrayLogEntry odomLog = LogFactory.getDoubleArray("8080/Localization/OdometryPose");
    private double[] lastBotpose = new double[6];
    private int ticksWithTags = 0;
    private Pose2d pose = new Pose2d(ZERO_TRANSFORM.getTranslation(), ZERO_TRANSFORM.getRotation());
    
    public Localizer(DriveSubsystem driveSubsystem, SwerveDriveOdometry odometry) {
        this.driveSubsystem = driveSubsystem;
        this.odometry = odometry;

        final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-tags");

        tv = table.getBooleanTopic("tv").subscribe(false);
        tl = table.getDoubleTopic("tl").subscribe(0.0);
        tid = table.getIntegerTopic("tid").subscribe(-1);
        botpose = table.getDoubleArrayTopic("botpose").subscribe(lastBotpose);
    }

    public Optional<Integer> getPrimaryApriltag() {
        return tv.get() ? Optional.of((int)tid.get()).filter(x -> x >= 1 && x <= 8) : Optional.empty();
    }

    public void periodic() {
        double currentTime = System.currentTimeMillis() / 1000.0;

        odomHistory.removeIf(p -> p.getSecond() < currentTime - 1.0); // Remove old positions
        Pose2d odometryPose = getOdometryPose();
        odomLog.append(new double[] {odometryPose.getX(), odometryPose.getY(), odometryPose.getRotation().getDegrees()});
        odomHistory.add(new Pair<>(odometryPose, currentTime));

        if (!DriveConstants.ENABLE_APRILTAGS) {
            pose = odometryPose;
            return;
        }

        if (!tv.get()) {
            if (ticksWithTags == 0) {
                odomHistory.clear();
                odometry.resetPosition(driveSubsystem.getGyro(), driveSubsystem.getPositions(), pose);
            }
            ticksWithTags = 0;
            return;
        }
        ticksWithTags++;

        double[] arr = botpose.get(lastBotpose);
        rawApriltagPoseLog.append(new double[] {arr[0], arr[1], arr[5]});
        lastBotpose = arr;
        Pose2d pose = new Pose2d(arr[0], arr[1], Rotation2d.fromDegrees(arr[5]));
        if (DriveConstants.ENABLE_GYRO_RECALIBRATION) {
            driveSubsystem.calibrateGyro(pose.getRotation());
        }
        pose = pose.plus(getLatencyCompensation((tl.get() + 11.0) / 1000.0));
        apriltagPoseLog.append(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
        if (DriveConstants.ENABLE_LATENCY_COMPENSATION) {
            this.pose = pose;
        }
    }

    public Pose2d getPose() {
        return tv.get() && DriveConstants.ENABLE_APRILTAGS ? pose : getOdometryPose();
    }

    private Pose2d getOdometryPose() {
        var pose = odometry.getPoseMeters();
        return new Pose2d(-pose.getX(), pose.getY(), driveSubsystem.getGyro());
    }

    private Transform2d getLatencyCompensation(double latency) {
        if (!DriveConstants.ENABLE_LATENCY_COMPENSATION) {
            return ZERO_TRANSFORM;
        }

        double time = (System.currentTimeMillis() / 1000.0) - latency;

        for (int i = 0; i < odomHistory.size() - 1; i++) {
            var prev = odomHistory.get(i);
            var next = odomHistory.get(i + 1);

            // Find 2 poses between target time
            if (prev.getSecond() > time || next.getSecond() < time) {
                continue;
            }

            double t = (time - prev.getSecond()) / (next.getSecond() - prev.getSecond());
            var p = prev.getFirst().interpolate(next.getFirst(), t);
            return new Transform2d(prev.getFirst(), p);
        }

        return ZERO_TRANSFORM;
    }
}
