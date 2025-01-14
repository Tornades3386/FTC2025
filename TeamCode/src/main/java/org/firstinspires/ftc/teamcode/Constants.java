package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

public class Constants {
    public static double CENTIMETER_PER_INCH = 2.54;
    public static double CENTIMETER_PER_INCH_INVERSE = 1 / CENTIMETER_PER_INCH;
    public static class DriveConstants {
        public static double MAX_MOVEMENT_PER_SECOND = 300;
        public static double MAX_ROTATION_PER_SECOND = 300;

        public static double[] MOTOR_PID = {1, 0, 0.0075};

        public static String FRONT_LEFT_MOTOR_NAME = "leftFrontBase";
        public static String FRONT_RIGHT_MOTOR_NAME = "rightFrontBase";
        public static String REAR_LEFT_MOTOR_NAME = "leftRearBase";
        public static String REAR_RIGHT_MOTOR_NAME = "rightRearBase";

        public static double GEAR_RATIO = 4 * 3;
        public static double ENCODER_COUNT_PER_REVOLUTION = 24;

        public static double WHEEL_DIAMETER = 3d * CENTIMETER_PER_INCH;
        public static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static double DISTANCE_TO_ROTATIONS = WHEEL_CIRCUMFERENCE / GEAR_RATIO;
        public static double DISTANCE_TO_TICKS = DISTANCE_TO_ROTATIONS / ENCODER_COUNT_PER_REVOLUTION;
        public static double DISTANCE_TO_TICKS_INVERSE = 1d / DISTANCE_TO_TICKS;
        public static double TICKS_TO_DISTANCE = ENCODER_COUNT_PER_REVOLUTION * GEAR_RATIO;
        public static double TICKS_TO_DISTANCE_INVERSE = 1d / TICKS_TO_DISTANCE;


        // Distance between centers of right and left wheels on robot
        public static double TRACK_WIDTH = 39.25;
        // Distance between front and back wheels on robot
        public static double WHEEL_BASE = 35.25;

        public static MecanumDriveKinematics DRIVE_KINEMATICS = new MecanumDriveKinematics(
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front left
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front right
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Rear left
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2)); // Rear right
    }
    public static class GrabberConstants {
        public static String GRABBER_LEFT_MOTOR_NAME = "leftGrabber";
        public static String GRABBER_RIGHT_MOTOR_NAME = "leftGrabber";
        public static String POIGNET_SERVO_NAME = "servoGrabberPoignet";
        public static String PINCE_SERVO_NAME = "servoGrabberPince";

    }
}