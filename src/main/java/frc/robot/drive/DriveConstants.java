package frc.robot.drive;

import edu.wpi.first.math.util.Units;

public final class DriveConstants {

    public static final double AZIMUTH_P = 20.0/360.0;
    public static final double AZIMUTH_D = 0.0/360.0;
    public static final double AZIMUTH_KS = 0.04 * 12; // Account for static friction
    public static final double DRIVE_KS = 0.04 * 12; // Account for static friction
    public static final double DRIVE_KV = 0.697;
    public static final double AZIMUTH_ERROR = 1.0;

    public static final double WIDTH = 0.5383022; // meters
    public static final double LENGTH = 0.6399022; // meters

    public static final double SPEED_BOOST = 1.75;
    public static final double MAX_SPEED = 6.35;
    public static final double MAX_ROTATION_SPEED = Math.PI * 1.5;

    public static final double TURN_TO_ERROR = 1.0; // degrees
    public static final double TURN_TO_MAX_SPEED = MAX_ROTATION_SPEED * 0.5;
    public static final double DRIVE_TO_ERROR = 0.05; // m
    public static final double DRIVE_TO_MAX_SPEED = MAX_SPEED * 0.5;

    public static final double ENCODER_FL_OFFSET = -370.895020 - 90;
    public static final double ENCODER_FR_OFFSET =  25.427333 + 180;
    public static final double ENCODER_BL_OFFSET = -96.001853 + 0;
    public static final double ENCODER_BR_OFFSET = -16.393339 + 0;

    public static final int AZIMUTH_FL_ID = 0;
    public static final int DRIVE_FL_ID = 1;
    public static final int ENCODER_A_FL = 0;
    public static final int ENCODER_B_FL = 1;
    public static final int ENCODER_FL_ID = 0;
    
    public static final int AZIMUTH_FR_ID = 2;
    public static final int DRIVE_FR_ID = 3;
    public static final int ENCODER_A_FR = 2;
    public static final int ENCODER_B_FR = 3;
    public static final int ENCODER_FR_ID = 3;
    
    public static final int AZIMUTH_BL_ID = 4;
    public static final int DRIVE_BL_ID = 5;
    public static final int ENCODER_A_BL = 4;
    public static final int ENCODER_B_BL = 5;
    public static final int ENCODER_BL_ID = 1;
    
    public static final int AZIMUTH_BR_ID = 6;
    public static final int DRIVE_BR_ID = 7;
    public static final int ENCODER_A_BR = 6;
    public static final int ENCODER_B_BR = 7;
    public static final int ENCODER_BR_ID = 2;

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.5 + (2 * 0.25)); // added tread height
    public static final double FALCON_500_ROTATIONS_TO_DISTANCE = 1.0 / 2048.0 * WHEEL_DIAMETER * Math.PI;
    public static final double AZIMUTH_GEAR_RATIO = 15.43 / 1.0;
    public static final double DRIVE_GEAR_RATIO = 6.54 / 1.0;

    public static final double ACCELERATION_TIME = 0.2;
    public static final double ROTATION_ACCELERATION_TIME = 0.1;
    public static final double MOTOR_ACCELERATION_TIME = 0.001;

    public static final boolean ENABLE_LATENCY_COMPENSATION = false;
    public static final boolean ENABLE_APRILTAGS = false;
    public static final boolean ENABLE_GYRO_RECALIBRATION = false;

    public static int FILTER_SIZE = 5;

    private DriveConstants() {}
}
