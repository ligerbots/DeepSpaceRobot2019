package org.ligerbots.robot;


public class RobotMap {
    public static final double YAW_ERROR_THRESHOLD = 0.5;

    public static final double ELEVATOR_ENCODER_TICKS_PER_REV = 1024; //Might be different. Change when u find out

    public static final double AUTO_TURN_ACCURACY_THRESHOLD = 1; //in degrees

    public static final double AUTO_DRIVE_DISTANCE_THRESHOLD = 24; //in inches

    public static final double SIDE_GEAR_RATIO = 12 / 72 * 44 / 72;

    public static final double CENTER_GEAR_RATIO = 1 / 12; //Not actually. Fix later

    public static final double SIDE_WHEEL_DIAMETER = 6.0;

    public static final double CENTER_WHEEL_DIAMETER = 4.0;

    public static final double WRIST_ENCODER_TICKS_PER_REV = 4096;

    public static final double INTAKE_IN_HEIGHT = 16; //ACTUALLY CHECK (UNITS IN WEIRD ELEVATOR HEIGHT)
  
    public static final int CT_INTAKE = 10; 

    public static final int PCM_ID = 12; 

    public static final double TICKS_TO_HEIGHT_COEFFICIENT = ELEVATOR_ENCODER_TICKS_PER_REV / (Math.PI * 0.6) * (54.0 / 18.0);

    public static final int DETERMINE_WHICH_ROBOT = 8; //Whether this deviceId is a talon will determine if we're controlling the second robot

    public static final int ABSOLUTE_ENCODER_CHANNEL = 1; //Not actually, fix later

    public static final int ABSOLUTE_ENCODER_OFFSET = 0;

    public static final boolean WRIST_USES_ABSOLUTE_ENCODER = true;

    public static final double SHAFT_DIAMETER = 0.6;
}
