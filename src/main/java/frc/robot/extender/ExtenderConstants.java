package frc.robot.extender;

public final class ExtenderConstants {

    public static final int FORWARD_SOLENOID_ID = 9; 
    public static final int REVERSE_SOLENOID_ID = 6;
    public static final int ULTRASONIC_ID = 0;
    
    public static final double ERROR = 7.0; // CM

    public static final double DISTANCE_RETRACTED = 0.0;
    public static final double DISTANCE_PICKUP = 0.0;
    public static final double DISTANCE_MIDDLE_ROW = 0.0;
    public static final double DISTANCE_TOP_ROW = 0.0;
    public static final double DISTANCE_DOUBLE_SUBSTATION = 0.0;

    public static final int FILTER_SIZE = 5;

    public static final double TIME_LOW = 0.25;
    public static final double TIME_MID = 0.5;

    public static final double EXTENDER_COAST_COMPENSTAION_TIME = 0.05;

    private ExtenderConstants() {}
}
