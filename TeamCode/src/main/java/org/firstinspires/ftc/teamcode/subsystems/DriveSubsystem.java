package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;


public class DriveSubsystem extends SubsystemBase {
    private static final DriveSubsystem INSTANCE = new DriveSubsystem();
    private static final MecanumDriveOdometry odometry = new MecanumDriveOdometry(DriveConstants.DRIVE_KINEMATICS, new Rotation2d());
    private static MotorEx frontLeft;

    private static MotorEx frontRight;
    private static MotorEx rearLeft;
    private static MotorEx rearRight;
    private static RevIMU imu;
    private static ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

    private DriveSubsystem() {
    }

    public static DriveSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();

        frontLeft = new MotorEx(globalSubsystem.hardwareMap, DriveConstants.FRONT_LEFT_MOTOR_NAME);
        frontRight = new MotorEx(globalSubsystem.hardwareMap, DriveConstants.FRONT_RIGHT_MOTOR_NAME);
        rearLeft = new MotorEx(globalSubsystem.hardwareMap, DriveConstants.REAR_LEFT_MOTOR_NAME);
        rearRight = new MotorEx(globalSubsystem.hardwareMap, DriveConstants.REAR_RIGHT_MOTOR_NAME);

        frontLeft.setVeloCoefficients(DriveConstants.MOTOR_PID[0], DriveConstants.MOTOR_PID[1], DriveConstants.MOTOR_PID[2]);
        frontRight.setVeloCoefficients(DriveConstants.MOTOR_PID[0], DriveConstants.MOTOR_PID[1], DriveConstants.MOTOR_PID[2]);
        rearLeft.setVeloCoefficients(DriveConstants.MOTOR_PID[0], DriveConstants.MOTOR_PID[1], DriveConstants.MOTOR_PID[2]);
        rearRight.setVeloCoefficients(DriveConstants.MOTOR_PID[0], DriveConstants.MOTOR_PID[1], DriveConstants.MOTOR_PID[2]);

        frontLeft.setDistancePerPulse(DriveConstants.TICKS_TO_DISTANCE_INVERSE);
        frontRight.setDistancePerPulse(DriveConstants.TICKS_TO_DISTANCE_INVERSE);
        rearLeft.setDistancePerPulse(DriveConstants.TICKS_TO_DISTANCE_INVERSE);
        rearRight.setDistancePerPulse(DriveConstants.TICKS_TO_DISTANCE_INVERSE);

        frontLeft.setInverted(true);
        rearRight.setInverted(true);

        frontLeft.setRunMode(Motor.RunMode.VelocityControl);
        frontRight.setRunMode(Motor.RunMode.VelocityControl);
        rearLeft.setRunMode(Motor.RunMode.VelocityControl);
        rearRight.setRunMode(Motor.RunMode.VelocityControl);

        imu = new RevIMU(globalSubsystem.hardwareMap, "navx");
        imu.invertGyro();
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative, Rotation2d rotateBy) {
        targetChassisSpeeds = fieldRelative ?
                fromFieldRelativeSpeeds(
                        chassisSpeeds, odometry.getPoseMeters().getRotation().plus(rotateBy))
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
    public void periodic() {}
    public void resetGyro() {
        imu.reset();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d newPose) {
        odometry.resetPosition(newPose, imu.getRotation2d());
    }
}
