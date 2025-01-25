package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RevIMU;

public class Drive9axisIMU extends SubsystemBase {
    private static final Drive9axisIMU INSTANCE = new Drive9axisIMU();

    private static final MecanumDriveOdometry odometry = new MecanumDriveOdometry(Constants.DriveConstants.DRIVE_KINEMATICS, new Rotation2d());
    private IMU imu;
    private Telemetry telemetry;
    private ImuOrientationOnRobot orientationOnRobot;
    private YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;
    private static MotorEx frontLeft;
    private static MotorEx frontRight;
    private static MotorEx rearLeft;
    private static MotorEx rearRight;
    private static ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

    private Drive9axisIMU() {
    }
    public static Drive9axisIMU getInstance() {
        return INSTANCE;
    }

    public void init() {
        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();
        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;


        orientationOnRobot = new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.RIGHT);
        imu = hardwareMap.get(IMU.class, "imu");
        //imu = new RevIMU(hardwareMap, "navx");
        imu.resetYaw();

        orientation = imu.getRobotYawPitchRollAngles();
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);

        frontLeft = new MotorEx(globalSubsystem.hardwareMap, Constants.DriveConstants.FRONT_LEFT_MOTOR_NAME);
        frontRight = new MotorEx(globalSubsystem.hardwareMap, Constants.DriveConstants.FRONT_RIGHT_MOTOR_NAME);
        rearLeft = new MotorEx(globalSubsystem.hardwareMap, Constants.DriveConstants.REAR_LEFT_MOTOR_NAME);
        rearRight = new MotorEx(globalSubsystem.hardwareMap, Constants.DriveConstants.REAR_RIGHT_MOTOR_NAME);

        frontLeft.setVeloCoefficients(Constants.DriveConstants.MOTOR_PID[0], Constants.DriveConstants.MOTOR_PID[1], Constants.DriveConstants.MOTOR_PID[2]);
        frontRight.setVeloCoefficients(Constants.DriveConstants.MOTOR_PID[0], Constants.DriveConstants.MOTOR_PID[1], Constants.DriveConstants.MOTOR_PID[2]);
        rearLeft.setVeloCoefficients(Constants.DriveConstants.MOTOR_PID[0], Constants.DriveConstants.MOTOR_PID[1], Constants.DriveConstants.MOTOR_PID[2]);
        rearRight.setVeloCoefficients(Constants.DriveConstants.MOTOR_PID[0], Constants.DriveConstants.MOTOR_PID[1], Constants.DriveConstants.MOTOR_PID[2]);

        frontLeft.setDistancePerPulse(Constants.DriveConstants.TICKS_TO_DISTANCE_INVERSE);
        frontRight.setDistancePerPulse(Constants.DriveConstants.TICKS_TO_DISTANCE_INVERSE);
        rearLeft.setDistancePerPulse(Constants.DriveConstants.TICKS_TO_DISTANCE_INVERSE);
        rearRight.setDistancePerPulse(Constants.DriveConstants.TICKS_TO_DISTANCE_INVERSE);

        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        rearLeft.setRunMode(Motor.RunMode.VelocityControl);
        rearRight.setRunMode(Motor.RunMode.VelocityControl);

    }
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, Rotation2d rotateBy) {
        targetChassisSpeeds = fieldRelative ?
                fromFieldRelativeSpeeds(
                        chassisSpeeds, odometry.getPoseMeters().getRotation().plus(rotateBy))
                : chassisSpeeds;
    }

    private ChassisSpeeds fromFieldRelativeSpeeds(ChassisSpeeds chassisSpeeds, Rotation2d rotateBy) {
        return new ChassisSpeeds(
                chassisSpeeds.vxMetersPerSecond * rotateBy.getCos() - chassisSpeeds.vyMetersPerSecond * rotateBy.getSin(),
                chassisSpeeds.vyMetersPerSecond * rotateBy.getCos() + chassisSpeeds.vxMetersPerSecond * rotateBy.getSin(),
                chassisSpeeds.omegaRadiansPerSecond
        );
    }

    @Override
    public void periodic() {
        Telemetry telemetry = GlobalSubsystem.getInstance().telemetry;
        telemetry.addLine("");
        orientation = imu.getRobotYawPitchRollAngles();
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        telemetry.addData("Yaw (Z)", JavaUtil.formatNumber(orientation.getYaw(AngleUnit.RADIANS), 2) + " Deg. (Heading)");
        telemetry.addData("Pitch (X)", JavaUtil.formatNumber(orientation.getPitch(AngleUnit.RADIANS), 2) + " Deg.");
        telemetry.addData("Roll (Y)", JavaUtil.formatNumber(orientation.getRoll(AngleUnit.RADIANS), 2) + " Deg.");
        telemetry.addLine("");
        telemetry.addData("Yaw (Z) velocity", JavaUtil.formatNumber(angularVelocity.zRotationRate, 2) + " Deg/Sec");
        telemetry.addData("Pitch (X) velocity", JavaUtil.formatNumber(angularVelocity.xRotationRate, 2) + " Deg/Sec");
        telemetry.addData("Roll (Y) velocity", JavaUtil.formatNumber(angularVelocity.yRotationRate, 2) + " Deg/Sec");


        telemetry.addData("Drive: Target Chassis X", targetChassisSpeeds.vxMetersPerSecond);
        telemetry.addData("Drive: Target Chassis Y", targetChassisSpeeds.vyMetersPerSecond);
        telemetry.addData("Drive: Target Chassis Rad", targetChassisSpeeds.omegaRadiansPerSecond);

        targetChassisSpeeds.vxMetersPerSecond = -targetChassisSpeeds.vxMetersPerSecond;

        MecanumDriveWheelSpeeds wheelSpeeds = Constants.DriveConstants.DRIVE_KINEMATICS.toWheelSpeeds(targetChassisSpeeds);


        telemetry.addData("Drive: Velocity frontLeft", frontLeft.getVelocity());
        telemetry.addData("Drive: Velocity frontRight", frontRight.getVelocity());
        telemetry.addData("Drive: Velocity rearLeft", rearLeft.getVelocity());
        telemetry.addData("Drive: Velocity rearRight", rearRight.getVelocity());

        telemetry.addData("Drive: IMU Rotation", Rotation2d.fromDegrees(orientation.getPitch(AngleUnit.DEGREES)));
        //telemetry.addData("Drive: IMU Raw Orientation", imu.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES));

        telemetry.addData("Drive: Calculated wheel speeds", wheelSpeeds);

        frontLeft.setVelocity(wheelSpeeds.frontLeftMetersPerSecond * Constants.DriveConstants.DISTANCE_TO_TICKS_INVERSE);
        frontRight.setVelocity(wheelSpeeds.frontRightMetersPerSecond * Constants.DriveConstants.DISTANCE_TO_TICKS_INVERSE);
        rearLeft.setVelocity(wheelSpeeds.rearLeftMetersPerSecond * Constants.DriveConstants.DISTANCE_TO_TICKS_INVERSE);
        rearRight.setVelocity(wheelSpeeds.rearRightMetersPerSecond * Constants.DriveConstants.DISTANCE_TO_TICKS_INVERSE);

        if (targetChassisSpeeds.vxMetersPerSecond != 0) {
            targetChassisSpeeds.vxMetersPerSecond = 0;
        }
        if (targetChassisSpeeds.vyMetersPerSecond != 0) {
            targetChassisSpeeds.vyMetersPerSecond = 0;
        }
        if (targetChassisSpeeds.omegaRadiansPerSecond != 0) {
            targetChassisSpeeds.omegaRadiansPerSecond = 0;
        }

        MecanumDriveWheelSpeeds realWheelSpeeds = new MecanumDriveWheelSpeeds(
                frontLeft.getVelocity() * Constants.DriveConstants.DISTANCE_TO_TICKS,
                frontRight.getVelocity() * Constants.DriveConstants.DISTANCE_TO_TICKS,
                rearLeft.getVelocity() * Constants.DriveConstants.DISTANCE_TO_TICKS,
                rearRight.getVelocity() * Constants.DriveConstants.DISTANCE_TO_TICKS
        );

        /*odometry.updateWithTime(
                GlobalSubsystem.getInstance().elapsedTime.seconds(),
                Rotation2d.fromDegrees(orientation.getPitch(AngleUnit.DEGREES)), realWheelSpeeds
        );

        ChassisSpeeds robotState = Constants.DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(realWheelSpeeds);

        Pose2d odometryPose = odometry.getPoseMeters();

        TelemetryPacket fieldPacket = GlobalSubsystem.getInstance().fieldPacket;

        fieldPacket.fieldOverlay().setTranslation(odometryPose.getY() * Constants.CENTIMETER_PER_INCH_INVERSE, odometryPose.getX() * Constants.CENTIMETER_PER_INCH_INVERSE);

        fieldPacket.fieldOverlay()
                .setStroke("blue")
                .setRotation(-odometryPose.getHeading())
                .strokeRect(-Constants.DriveConstants.TRACK_WIDTH / 2, -Constants.DriveConstants.WHEEL_BASE / 2, Constants.DriveConstants.TRACK_WIDTH, Constants.DriveConstants.WHEEL_BASE);

        fieldPacket.fieldOverlay().setStroke("green")
                .strokeLine(0, 0, -robotState.vyMetersPerSecond, 0);

        fieldPacket.fieldOverlay().setStroke("red")
                .strokeLine(0, 0, 0, -robotState.vxMetersPerSecond);

        fieldPacket.fieldOverlay().setStroke("white").strokeLine(0, 0, -robotState.vyMetersPerSecond, -robotState.vxMetersPerSecond);

        telemetry.addData("Drive: Estimated Pose", odometryPose);*/
    }

}
