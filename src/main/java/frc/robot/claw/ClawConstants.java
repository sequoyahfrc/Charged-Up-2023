package frc.robot.claw;

public final class ClawConstants {

    public static final int INTAKE_L_ID = 10;
    public static final int INTAKE_R_ID = 11;
    public static final int WRIST_ID = 12;

    public static final double INTAKE_SPEED = 0.15;
    public static final double SHOOT_SPEED = -1.0;

    // Speed (in percent) required to hold the arm at the horizontal
    public static final double HORIZONTAL_KS = 0.06;
    public static final double HORIZONTAL_ANGLE = -80.0;
    public static final double CLAW_MAX_ANGLE = 0;

    public static final double WRIST_P = 2.0 / 360.0;
    // PID error, in degrees
    public static final double WRIST_ERROR = 3.0;
    public static final double WRIST_GEAR_RATIO = 48.0 / 1.0;

    public static final double WRIST_MANUAL_UP_SPEED = 0.15;
    public static final double WRIST_MANUAL_DOWN_SPEED = -0.15;
    
    public static final double SHOOT_TIME = 1.0;
    public static final double SHOOT_DELAY = 0.5;

    public static final double WRIST_ACCELERATION_TIME = 0.1;

    private ClawConstants() {}
}
