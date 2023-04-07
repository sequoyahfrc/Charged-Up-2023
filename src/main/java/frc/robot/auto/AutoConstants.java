package frc.robot.auto;

public final class AutoConstants {

    public static final String ROUTINE_NOTHING = "Nothing";
    public static final String ROUTINE_BALANCE = "Balance";
    public static final String ROUTINE_MOBILITY = "Mobility";
    public static final String ROUTINE_HIGH_CUBE = "High Cube";
    public static final String ROUTINE_MID_CUBE = "Mid Cube";
    public static final String ROUTINE_HIGH_CUBE_BALANCE = "High Cube + Balance";
    public static final String ROUTINE_MID_CUBE_BALANCE = "Mid Cube + Balance";
    public static final String ROUTINE_HIGH_CUBE_MOBILITY_SHORT_SIDE = "High Cube + Mobility (Short Side)";
    public static final String ROUTINE_MID_CUBE_MOBILITY_SHORT_SIDE = "Mid Cube + Mobility (Short Side)";
    public static final String ROUTINE_HIGH_CUBE_MOBILITY_LONG_SIDE = "High Cube + Mobility (Long Side)";
    public static final String ROUTINE_MID_CUBE_MOBILITY_LONG_SIDE = "Mid Cube + Mobility (Long Side)";
    public static final String[] ROUTINES = new String[] {
        ROUTINE_NOTHING,
        ROUTINE_BALANCE,
        ROUTINE_MOBILITY,
        ROUTINE_HIGH_CUBE,
        ROUTINE_MID_CUBE,
        ROUTINE_HIGH_CUBE_BALANCE,
        ROUTINE_MID_CUBE_BALANCE,
        ROUTINE_HIGH_CUBE_MOBILITY_SHORT_SIDE,
        ROUTINE_MID_CUBE_MOBILITY_SHORT_SIDE,
        ROUTINE_HIGH_CUBE_MOBILITY_LONG_SIDE,
        ROUTINE_MID_CUBE_MOBILITY_LONG_SIDE
    };

    public static final double MOBILITY_TIME = 4.0;
    public static final double MOBILITY_SPEED = -1.75;

    public static final double BALANCE_TILT_BACK_SPEED = -2.75;
    public static final double BALANCE_TILT_BACK_THRESHOLD = -7.5; // degrees
    public static final double BALANCE_TILT_FORWARD_SPEED = -1.0;
    public static final double BALANCE_TILT_FORWARD_THRESHOLD = -11; // degrees
    public static final double BALANCE_CORRECTING_SPEED = 1.0;
    public static final double BALANCE_CORRECTING_TIME = 0.25;

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