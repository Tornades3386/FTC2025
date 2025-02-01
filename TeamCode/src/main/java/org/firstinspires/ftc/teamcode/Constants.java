package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;

public class Constants {
    public static double CENTIMETER_PER_INCH = 2.54;
    public static double CENTIMETER_PER_INCH_INVERSE = 1 / CENTIMETER_PER_INCH;
    public static class DriveConstants {
        public static double MAX_MOVEMENT_PER_SECOND = 100;
        public static double MAX_ROTATION_PER_SECOND = 100;

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
        public static String GRABBER_DROP_LIMIT = "grabberDropLimit";
        public static double POIGNET_SERVO_MIN_POSITION = 0;
        public static double POIGNET_SERVO_MAX_POSITION = 0.75;
        public static double POIGNET_SERVO_MID_POSITION = 0.35;

    }
    public static class ElevatorConstants {
        public static String ELEVATOR_LEFT_MOTOR_NAME = "leftElevator";
        public static String ELEVATOR_RIGHT_MOTOR_NAME = "rightElevator";
        public static String ELEVATOR_DOWN_LIMIT = "downElevator";
        public static String ELEVATOR_PINCE_SERVO_NAME = "elevator0";
        public static String ELEVATOR_COUDE_SERVO_NAME = "elevator1";
        public static String ELEVATOR_POIGNET_SERVO_NAME = "elevator2";

        public static double PINCE_SERVO_MIN_POSITION = 0.17;
        public static double PINCE_SERVO_MAX_POSITION = 0.25;

        public static double COUDE_SERVO_MIN_POSITION = 0.17;
        public static double COUDE_SERVO_MAX_POSITION = 1;

        public static double POIGNET_SERVO_MIN_POSITION = 0.83;
        public static double POIGNET_SERVO_MAX_POSITION = 0.93;


    }
}