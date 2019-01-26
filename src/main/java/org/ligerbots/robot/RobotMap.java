package org.ligerbots.robot;


public class RobotMap {
    public static final double YAW_ERROR_THRESHOLD = 0.5;

    public static final double ELEVATOR_ENCODER_TICKS_PER_REV = 1024; //Might be different. Change when u find out

    public static final double AUTO_TURN_ACCURACY_THRESHOLD = 1; //in degrees

    public static final double AUTO_DRIVE_DISTANCE_THRESHOLD = 2; //in inches

    public static final double AUTO_DRIVE_STARTALIGN_THRESHOLD = 20; //in inches

    public static final double SIDE_GEAR_RATIO = 12 / 72 * 44 / 72;

    public static final double CENTER_GEAR_RATIO = 1 / 12; //Not actually. Fix later

    public static final double SIDE_WHEEL_DIAMETER = 6.0;

    public static final double CENTER_WHEEL_DIAMETER = 4.0;

}
