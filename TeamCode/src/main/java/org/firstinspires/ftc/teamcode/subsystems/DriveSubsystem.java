package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;


public class DriveSubsystem extends SubsystemBase {
    private static final DriveSubsystem INSTANCE = new DriveSubsystem();
    private static final MecanumDriveOdometry odometry = new MecanumDriveOdometry(DriveConstants.DRIVE_KINEMATICS, new Rotation2d());
    private static class Base {
        private static MotorEx frontLeft;
        private static MotorEx frontRight;
        private static MotorEx rearLeft;
        private static MotorEx rearRight;
    }

    private final double teta = Math.toRadians(45);
    double epsilon = 0.00001;
    double factor = 1/Math.abs(Math.sin(teta));
//    private static IMU imu;
//    private static NavxMicroNavigationSensor navx;
    private static RevIMU imu;
    boolean isPOV = false;
    private static ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

    Base base = new Base();
    private DriveSubsystem() {
    }

    public static DriveSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {

        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();
        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(
//                new IMU.Parameters(
//                        new RevHubOrientationOnRobot(
//                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
//                        )
//                )
//        );
        imu = new RevIMU(hardwareMap, "navx");

//        imu.resetYaw();
        imu.reset();

        Base.frontLeft = new MotorEx(globalSubsystem.hardwareMap, DriveConstants.FRONT_LEFT_MOTOR_NAME);
        Base.frontRight = new MotorEx(globalSubsystem.hardwareMap, DriveConstants.FRONT_RIGHT_MOTOR_NAME);
        Base.rearLeft = new MotorEx(globalSubsystem.hardwareMap, DriveConstants.REAR_LEFT_MOTOR_NAME);
        Base.rearRight = new MotorEx(globalSubsystem.hardwareMap, DriveConstants.REAR_RIGHT_MOTOR_NAME);

        /*Base.frontLeft.setVeloCoefficients(DriveConstants.MOTOR_PID[0], DriveConstants.MOTOR_PID[1], DriveConstants.MOTOR_PID[2]);
        Base.frontRight.setVeloCoefficients(DriveConstants.MOTOR_PID[0], DriveConstants.MOTOR_PID[1], DriveConstants.MOTOR_PID[2]);
        Base.rearLeft.setVeloCoefficients(DriveConstants.MOTOR_PID[0], DriveConstants.MOTOR_PID[1], DriveConstants.MOTOR_PID[2]);
        Base.rearRight.setVeloCoefficients(DriveConstants.MOTOR_PID[0], DriveConstants.MOTOR_PID[1], DriveConstants.MOTOR_PID[2]);

        Base.frontLeft.setDistancePerPulse(DriveConstants.TICKS_TO_DISTANCE_INVERSE);
        Base.frontRight.setDistancePerPulse(DriveConstants.TICKS_TO_DISTANCE_INVERSE);
        Base.rearLeft.setDistancePerPulse(DriveConstants.TICKS_TO_DISTANCE_INVERSE);
        Base.rearRight.setDistancePerPulse(DriveConstants.TICKS_TO_DISTANCE_INVERSE);*/

        Base.frontLeft.setRunMode(Motor.RunMode.RawPower);
        Base.frontRight.setRunMode(Motor.RunMode.RawPower);
        Base.rearLeft.setRunMode(Motor.RunMode.RawPower);
        Base.rearRight.setRunMode(Motor.RunMode.RawPower);

        Base.frontLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        Base.frontLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        Base.frontLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        Base.frontLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);


        /*frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        rearLeft.setRunMode(Motor.RunMode.VelocityControl);
        rearRight.setRunMode(Motor.RunMode.VelocityControl);*/

        //imu = hardwareMap.get(RevIMU.class, "navx");
        //imu = new RevIMU(globalSubsystem.hardwareMap, "navx");

        //imu.invertGyro();
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, Rotation2d rotateBy) {
        targetChassisSpeeds = fieldRelative ?
                fromFieldRelativeSpeeds(
                        chassisSpeeds, imu.getRotation2d())
                : chassisSpeeds;
    }

    private ChassisSpeeds fromFieldRelativeSpeeds(ChassisSpeeds chassisSpeeds, Rotation2d robotAngle) {
        return new ChassisSpeeds(
                chassisSpeeds.vxMetersPerSecond * robotAngle.getCos() - chassisSpeeds.vyMetersPerSecond * robotAngle.getSin(),
                chassisSpeeds.vyMetersPerSecond * robotAngle.getCos() + chassisSpeeds.vxMetersPerSecond * robotAngle.getSin(),
                chassisSpeeds.omegaRadiansPerSecond
        );
    }

    @Override
    public void periodic() {
        GlobalSubsystem.getInstance().telemetry.addData("NavX", imu.getRotation2d());
        // Get Robot Orientation
        /*double turn = -gamepad1.right_stick_x;
        Point dir_vector = new Point(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        Orientation myRobotOrientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
        );
        double pov_angle = myRobotOrientation.thirdAngle;

        if(pov_angle < 0){
            pov_angle = 360-Math.abs(pov_angle);
        }
        double a = 45;
        if(pov_angle >= a && pov_angle <= 180-a){
            pov_angle += 180;
        } else if(pov_angle >= 270-a && pov_angle <= 270+a){
            pov_angle -= 180;
        }
        pov_angle = Math.toRadians(pov_angle);
        dir_vector = changementDeBase(dir_vector, pov_angle);*/
    }
    /*public void resetGyro() {
        imu.reset();
    }*/

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /*public void resetPose(Pose2d newPose) {
        odometry.resetPosition(newPose, imu.getRotation2d());
    }*/

}
