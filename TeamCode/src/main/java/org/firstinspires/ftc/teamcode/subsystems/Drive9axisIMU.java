package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Constants;

public class Drive9axisIMU extends SubsystemBase {
    private static final Drive9axisIMU INSTANCE = new Drive9axisIMU();

    private static final MecanumDriveOdometry odometry = new MecanumDriveOdometry(Constants.DriveConstants.DRIVE_KINEMATICS, new Rotation2d());
    private IMU imu;
    private Telemetry telemetry;
    private ImuOrientationOnRobot orientationOnRobot;
    private YawPitchRollAngles orientation;
    AngularVelocity angularVelocity;
    private double botHeading;
    private double frontLeftPower;
    private double backLeftPower;
    private double frontRightPower;
    private double backRightPower;
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

        imu = hardwareMap.get(IMU.class, "imu");
        //orientationOnRobot = new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.FORWARD);
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        //imu = new RevIMU(hardwareMap, "navx");
        imu.resetYaw();

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        frontLeft = new MotorEx(globalSubsystem.hardwareMap, Constants.DriveConstants.FRONT_LEFT_MOTOR_NAME);
        frontRight = new MotorEx(globalSubsystem.hardwareMap, Constants.DriveConstants.FRONT_RIGHT_MOTOR_NAME);
        rearLeft = new MotorEx(globalSubsystem.hardwareMap, Constants.DriveConstants.REAR_LEFT_MOTOR_NAME);
        rearRight = new MotorEx(globalSubsystem.hardwareMap, Constants.DriveConstants.REAR_RIGHT_MOTOR_NAME);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        rearLeft.setRunMode(Motor.RunMode.RawPower);
        rearRight.setRunMode(Motor.RunMode.RawPower);
        

        frontLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

    }
    public void drive(double letfY, double LeftX, double RightX, boolean fieldRelative) {
        double rx;
        double rotY;
        double rotX;
        if (!fieldRelative){
            rotX = LeftX * 1.1;
            rotY = -letfY;
            rx = RightX;
            givePower(rotX, rotY, rx);

        }else{
            rotX = (LeftX* Math.cos(-botHeading) - letfY * Math.sin(-botHeading)) *1.1;
            rotY = -(letfY * Math.sin(-botHeading) + LeftX * Math.cos(-botHeading));
            rx = RightX;
            givePower(rotX, rotY, rx);
        }
    }

    public void givePower(double rotX, double rotY, double rx){
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
    }



    @Override
    public void periodic() {
        Telemetry telemetry = GlobalSubsystem.getInstance().telemetry;
        telemetry.addLine("");
        orientation = imu.getRobotYawPitchRollAngles();
        angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);
        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        frontLeft.set(frontLeftPower);
        rearLeft.set(backLeftPower);
        frontRight.set(frontRightPower);
        rearRight.set(backRightPower);


        /*telemetry.addData("Yaw (Z)", JavaUtil.formatNumber(orientation.getYaw(AngleUnit.RADIANS), 2) + " Deg. (Heading)");
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
        );*/

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
