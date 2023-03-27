package frc.robot.elevator;

public final class ElevatorConstants {

    public static final int LEFT_MOTOR_ID = 9;
    public static final int RIGHT_MOTOR_ID = 8;
    public static final int BOTTOM_LIMIT_SWITCH_ID = 8;
    public static final int TOP_LIMIT_SWITCH_ID = 9;

    public static final double P = 0.2;
    public static final double D = 0.0;
    public static final double DEADBAND = 0.05;
    public static final double KS = 0.1;

    public static final double MANUAL_UP_SPEED = 0.375;
    public static final double MANUAL_DOWN_SPEED = -0.13;
    public static final double MIN_SPEED = 0.01;
    public static final double ZERO_SPEED = -0.1;
    public static final double MAX_SPEED = 0.4;

    public static final double MAX_HEIGHT = 43.0;

    public static final double ACCELERATION_TIME = 0.1;
}
