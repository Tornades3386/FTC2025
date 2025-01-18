package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private static final ElevatorSubsystem INSTANCE = new ElevatorSubsystem();
    private MotorEx elevatorRightMotor;
    private MotorEx elevatorLeftMotor;
    private Telemetry telemetry;
    private DigitalChannel elevatorDownLimit;
    private ServoEx El0Servo, El1Servo, El2Servo;
    private double El0ServoCurrentPosition, El1ServoCurrentPosition,El2ServoCurrentPosition;
    private ElevatorSubsystem() {}
    public static ElevatorSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();
        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;

        telemetry = globalSubsystem.telemetry;

        elevatorRightMotor = new MotorEx(globalSubsystem.hardwareMap, Constants.ElevatorConstants.ELEVATOR_RIGHT_MOTOR_NAME);
        elevatorLeftMotor = new MotorEx(globalSubsystem.hardwareMap, Constants.ElevatorConstants.ELEVATOR_LEFT_MOTOR_NAME);

        El0Servo = new SimpleServo(hardwareMap, Constants.ElevatorConstants.ELEVATOR_0_SERVO_NAME, 0.25, 1);
        El1Servo = new SimpleServo(hardwareMap, Constants.ElevatorConstants.ELEVATOR_1_SERVO_NAME, 0.25, 1);
        El2Servo = new SimpleServo(hardwareMap, Constants.ElevatorConstants.ELEVATOR_2_SERVO_NAME, 0.25, 1);

        El0ServoCurrentPosition = 0.5;
        El1ServoCurrentPosition = 0.5;
        El2ServoCurrentPosition = 0.5;

        El0Servo.setPosition(El0ServoCurrentPosition );
        El1Servo.setPosition(El1ServoCurrentPosition );
        El2Servo.setPosition(El2ServoCurrentPosition );


        elevatorRightMotor.setRunMode(Motor.RunMode.RawPower);
        elevatorRightMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        elevatorLeftMotor.setRunMode(Motor.RunMode.RawPower);
        elevatorLeftMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        elevatorRightMotor.setInverted(false);
        elevatorLeftMotor.setInverted(true);

        elevatorRightMotor.resetEncoder();
        elevatorLeftMotor.resetEncoder();


        elevatorDownLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, Constants.ElevatorConstants.ELEVATOR_DOWN_LIMIT);
        //bottomLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, ArmConstants.BOTTOM_LIMIT_SWITCH_NAME);

        elevatorDownLimit.setMode(DigitalChannel.Mode.INPUT);

    }

    public void setElevator(double power) {

        if (power<0 & !elevatorDownLimit.getState()) {
            elevatorRightMotor.stopMotor();
            elevatorLeftMotor.stopMotor();
        }else{
            elevatorRightMotor.set(power);
            elevatorLeftMotor.set(power);
        }

    }
    public void stopElevator() {

        elevatorRightMotor.set(0);
        elevatorLeftMotor.set(0);
    }

    public void setServo(){

    }


    public void periodic() {
        telemetry.addData("Elevator Left", elevatorLeftMotor.getCurrentPosition());
        telemetry.addData("Elevator Right", elevatorRightMotor.getCurrentPosition());
        //telemetry.addData("Pince", pinceServo.get );
        //pinceServo.setPower(0.3);


    }
}
