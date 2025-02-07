package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private static final ClimbSubsystem INSTANCE = new ClimbSubsystem();
    private MotorEx grabberRightMotor;
    private MotorEx grabberLeftMotor;
    private MotorEx elevatorRightMotor;
    private MotorEx elevatorLeftMotor;
    private MotorGroup elevatorMotors;
    private CRServo pinceServo;
    private ServoEx poignetServo;
    private Telemetry telemetry;
    private MotorGroup slideMotors;

    private ClimbSubsystem() {
    }

    public static ClimbSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();
        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;

        telemetry = globalSubsystem.telemetry;

        grabberRightMotor = new MotorEx(globalSubsystem.hardwareMap, Constants.GrabberConstants.GRABBER_LEFT_MOTOR_NAME);
        grabberLeftMotor = new MotorEx(globalSubsystem.hardwareMap, Constants.GrabberConstants.GRABBER_RIGHT_MOTOR_NAME);


        grabberRightMotor.setRunMode(Motor.RunMode.RawPower);
        grabberRightMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        grabberLeftMotor.setRunMode(Motor.RunMode.RawPower);
        grabberLeftMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);


        grabberRightMotor.setInverted(false);
        grabberLeftMotor.setInverted(true);

        grabberRightMotor.resetEncoder();


        slideMotors = new MotorGroup(grabberLeftMotor, grabberRightMotor);
    }
}
