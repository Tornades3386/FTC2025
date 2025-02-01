package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.COUDE_SERVO_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.COUDE_SERVO_MIN_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.ELEVATOR_COUDE_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.ELEVATOR_PINCE_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.ELEVATOR_POIGNET_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.PINCE_SERVO_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.PINCE_SERVO_MIN_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.POIGNET_SERVO_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.POIGNET_SERVO_MIN_POSITION;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
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
    private ServoEx ElPinceServo, ElCoudeServo, ElPoignetServo;
    private double ElPinceServoCurrentPosition, ElCoudeServoCurrentPosition,ElPoignetServoCurrentPosition;
    private boolean PoignetZero, PinceZero, CoudeZero;
    private MotorGroup elevatorMotors;
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

        ElPinceServo = new SimpleServo(hardwareMap, ELEVATOR_PINCE_SERVO_NAME, PINCE_SERVO_MIN_POSITION, PINCE_SERVO_MAX_POSITION);
        ElCoudeServo = new SimpleServo(hardwareMap, ELEVATOR_COUDE_SERVO_NAME, COUDE_SERVO_MIN_POSITION, COUDE_SERVO_MAX_POSITION);
        ElPoignetServo = new SimpleServo(hardwareMap, ELEVATOR_POIGNET_SERVO_NAME, POIGNET_SERVO_MIN_POSITION, POIGNET_SERVO_MAX_POSITION);

        ElPinceServoCurrentPosition = PINCE_SERVO_MIN_POSITION;
        ElCoudeServoCurrentPosition = COUDE_SERVO_MIN_POSITION;
        ElPoignetServoCurrentPosition = POIGNET_SERVO_MAX_POSITION;

        ElPinceServo.setPosition(ElPinceServoCurrentPosition );
        ElCoudeServo.setPosition(ElCoudeServoCurrentPosition );
        ElPoignetServo.setPosition(ElPoignetServoCurrentPosition );


        elevatorRightMotor.setRunMode(Motor.RunMode.RawPower);
        elevatorRightMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        elevatorLeftMotor.setRunMode(Motor.RunMode.RawPower);
        elevatorLeftMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        elevatorRightMotor.setInverted(false);
        elevatorLeftMotor.setInverted(true);

        elevatorRightMotor.resetEncoder();

        elevatorMotors = new MotorGroup(elevatorLeftMotor, elevatorRightMotor);


        elevatorDownLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, Constants.ElevatorConstants.ELEVATOR_DOWN_LIMIT);
        //bottomLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, ArmConstants.BOTTOM_LIMIT_SWITCH_NAME);

        elevatorDownLimit.setMode(DigitalChannel.Mode.INPUT);
        PoignetZero = false;
        PinceZero = true;
        CoudeZero = true;

    }

    public void setElevator(double power) {

        if (power<0 & !elevatorDownLimit.getState()) {
            elevatorMotors.stopMotor();
        }else{
            elevatorMotors.set(power);
        }

    }
    public void stopElevator() {
        elevatorMotors.stopMotor();
    }


    public void setPoignet() {
        if (PoignetZero){
            ElPoignetServoCurrentPosition = POIGNET_SERVO_MAX_POSITION;
        }else{
            ElPoignetServoCurrentPosition = POIGNET_SERVO_MIN_POSITION;
        }
    }

    public void setPince() {
        if (!PinceZero){
            ElPinceServoCurrentPosition = PINCE_SERVO_MAX_POSITION;
        }else{
            ElPinceServoCurrentPosition = PINCE_SERVO_MIN_POSITION;
        }
    }

    public void setCoude() {
        if (!PinceZero){
            ElCoudeServoCurrentPosition = COUDE_SERVO_MAX_POSITION;
        }else{
            ElCoudeServoCurrentPosition = COUDE_SERVO_MIN_POSITION;
        }
    }


    public void periodic() {
        telemetry.addData("Elevator Left", elevatorLeftMotor.getCurrentPosition());
        telemetry.addData("Elevator Right", elevatorRightMotor.getCurrentPosition());
        //telemetry.addData("Pince", pinceServo.get );
        //pinceServo.setPower(0.3);

        ElPinceServo.setPosition(ElPinceServoCurrentPosition);
        ElCoudeServo.setPosition(ElCoudeServoCurrentPosition);
        ElPoignetServo.setPosition(ElPoignetServoCurrentPosition);


        if (!elevatorDownLimit.getState()) {
            elevatorMotors.stopMotor();
        }

    }


    public void startClimb() {

        // Grabber Goes to limit switch and poignet goes up
        elevatorMotors.set(-0.4);
        //Elevator Goes up

    }
}
