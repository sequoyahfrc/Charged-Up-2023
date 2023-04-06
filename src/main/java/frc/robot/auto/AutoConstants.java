package frc.robot.auto;

public final class AutoConstants {

    public static final String ROUTINE_NOTHING = "Nothing";
    public static final String ROUTINE_BALANCE = "Balance";
    public static final String ROUTINE_MOBILITY = "Mobility";
    public static final String ROUTINE_HIGH_CUBE = "High Cube";
    public static final String ROUTINE_MID_CUBE = "Mid Cube";
    public static final String ROUTINE_HIGH_CUBE_BALANCE = "High Cube + Balance";
    public static final String ROUTINE_MID_CUBE_BALANCE = "Mid Cube + Balance";
    public static final String[] ROUTINES = new String[] {
        ROUTINE_NOTHING,
        ROUTINE_BALANCE,
        ROUTINE_MOBILITY,
        ROUTINE_HIGH_CUBE,
        ROUTINE_MID_CUBE,
        ROUTINE_HIGH_CUBE_BALANCE,
        ROUTINE_MID_CUBE_BALANCE
    };

    public static final double MOBILITY_TIME = 4.0;
    public static final double MOBILITY_SPEED = -1.75;

    public static final double BALANCE_TILT_BACK_SPEED = -4.0;
    public static final double BALANCE_TILT_BACK_THRESHOLD = -10; // degrees
    public static final double BALANCE_TILT_FORWARD_SPEED = -1.3;
    public static final double BALANCE_TILT_FORWARD_THRESHOLD = -11; // degrees
    public static final double BALANCE_CORRECTING_SPEED = 1.85;
    public static final double BALANCE_CORRECTING_THRESHOLD = 2.0; // degrees

    public static final double BALANCE2_MOBILITY_SPEED = -2;
    public static final double BALANCE2_MOBILITY_X_THRESHOLD = 5.5;

    public static final double BALANCE_VELOCITY_THRESHOLD = 11.2;
    public static final int BALANCE_FILTER_SIZE = 3;

    public static final double HIGH_CUBE_HEIGHT = 8.08;
    public static final double HIGH_CUBE_ANGLE = 57.86;
    public static final double HIGH_CUBE_SPEED = -0.4;

    public static final double LEFT_CONE_TX = -24.052881;

    private AutoConstants() {}
}