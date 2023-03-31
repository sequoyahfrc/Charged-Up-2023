package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;

public final class AutoConstants {

    private static final double INCHES_TO_METERS = 2.54 / 100.0;
    public static final double FIELD_CENTER_X_INCHES = 325.625;
    public static final double FIELD_CENTER_Y_INCHES = 157.8;
    public static final double CENTER_LINE_TO_PIECES = 47.36;
    public static final double PIECES_TO_BOTTOM_COMMUNITY_TAPE = 85.13;
    public static final double TOP_COMMUNITY_TAPE_BLUE = FIELD_CENTER_X_INCHES - CENTER_LINE_TO_PIECES - PIECES_TO_BOTTOM_COMMUNITY_TAPE;
    public static final double BOTTOM_COMMUNITY_TAPE_BLUE = FIELD_CENTER_X_INCHES - CENTER_LINE_TO_PIECES - PIECES_TO_BOTTOM_COMMUNITY_TAPE - 62.0625;
    public static final double TOP_COMMUNITY_TAPE_RED = FIELD_CENTER_X_INCHES + CENTER_LINE_TO_PIECES + PIECES_TO_BOTTOM_COMMUNITY_TAPE;
    public static final double BOTTOM_COMMUNITY_TAPE_RED = FIELD_CENTER_X_INCHES + CENTER_LINE_TO_PIECES + PIECES_TO_BOTTOM_COMMUNITY_TAPE + 62.0625;
    public static final double GAME_PIECE_1_Y = 36.19;
    public static final double GAME_PIECE_2_Y = GAME_PIECE_1_Y + 48;
    public static final double GAME_PIECE_3_Y = GAME_PIECE_2_Y + 48;
    public static final double GAME_PIECE_4_Y = GAME_PIECE_3_Y + 48;

    public static final Translation2d GAME_PIECE_1_BLUE = new Translation2d(FIELD_CENTER_X_INCHES - CENTER_LINE_TO_PIECES, GAME_PIECE_1_Y).times(INCHES_TO_METERS);
    public static final Translation2d GAME_PIECE_2_BLUE = new Translation2d(FIELD_CENTER_X_INCHES - CENTER_LINE_TO_PIECES, GAME_PIECE_2_Y).times(INCHES_TO_METERS);
    public static final Translation2d GAME_PIECE_3_BLUE = new Translation2d(FIELD_CENTER_X_INCHES - CENTER_LINE_TO_PIECES, GAME_PIECE_3_Y).times(INCHES_TO_METERS);
    public static final Translation2d GAME_PIECE_4_BLUE = new Translation2d(FIELD_CENTER_X_INCHES - CENTER_LINE_TO_PIECES, GAME_PIECE_4_Y).times(INCHES_TO_METERS);

    public static final Translation2d GAME_PIECE_1_RED = new Translation2d(FIELD_CENTER_X_INCHES + CENTER_LINE_TO_PIECES, GAME_PIECE_1_Y).times(INCHES_TO_METERS);
    public static final Translation2d GAME_PIECE_2_RED = new Translation2d(FIELD_CENTER_X_INCHES + CENTER_LINE_TO_PIECES, GAME_PIECE_2_Y).times(INCHES_TO_METERS);
    public static final Translation2d GAME_PIECE_3_RED = new Translation2d(FIELD_CENTER_X_INCHES + CENTER_LINE_TO_PIECES, GAME_PIECE_3_Y).times(INCHES_TO_METERS);
    public static final Translation2d GAME_PIECE_4_RED = new Translation2d(FIELD_CENTER_X_INCHES + CENTER_LINE_TO_PIECES, GAME_PIECE_4_Y).times(INCHES_TO_METERS);

    public static final String ROUTINE_NOTHING = "Nothing";
    public static final String ROUTINE_BALANCE = "Balance";
    public static final String ROUTINE_BALANCE_INVERTED = "Balance Backwards";
    public static final String ROUTINE_BALANCE2 = "Mobility + Balance";
    public static final String ROUTINE_MOBILITY = "Mobility";
    public static final String ROUTINE_DUMP = "Dump";
    public static final String ROUTINE_DUMP2 = "Dump & Balance";
    public static final String[] ROUTINES = new String[] {
        ROUTINE_NOTHING,
        ROUTINE_BALANCE,
        ROUTINE_BALANCE_INVERTED,
        ROUTINE_BALANCE2,
        ROUTINE_MOBILITY,
        ROUTINE_DUMP,
        ROUTINE_DUMP2,
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

    public static final double HIGH_CUBE_HEIGHT = 26;
    public static final double HIGH_CUBE_ANGLE = 44;

    private AutoConstants() {}
}