package frc.robot.claw;

public final class ClawConstants {

    public static final int INTAKE_L_ID = 10;
    public static final int INTAKE_R_ID = 11;
    public static final int WRIST_ID = 12;

    public static final double INTAKE_SPEED = -0.3;
    public static final double SHOOT_SPEED = 1.0;

    // Speed (in percent) required to hold the arm at the horizontal
    // TODO: determine horizontal ks if needed, otherwise set to 0 
    public static final double HORIZONTAL_KS = 0.0;
    // TODO: determine based starting position, also in degrees
    public static final double HORIZONTAL_ANGLE = 0.0;

    // TODO: determine
    public static final double WRIST_P = 1.0;
    // PID error, in degrees
    // TODO: determine
    public static final double WRIST_ERROR = 1.0;
    public static final double WRIST_GEAR_RATIO = 48.0 / 1.0;

    // TODO: may need tweaking
    public static final double WRIST_MANUAL_UP_SPEED = 1.0;
    public static final double WRIST_MANUAL_DOWN_SPEED = -1.0;
    
    public static final double SHOOT_TIME = 1.0;
    public static final double SHOOT_DELAY = 0.5;

    public static final double WRIST_ACCELERATION_TIME = 0.1;

    private ClawConstants() {}
}
