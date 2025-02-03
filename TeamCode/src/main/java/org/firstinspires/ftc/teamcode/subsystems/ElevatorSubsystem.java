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
    private MotorEx elevatorRightMotor, elevatorLeftMotor, elevatorRightMotorEncoder;
    private Telemetry telemetry;
    private DigitalChannel elevatorDownLimit;
    private ServoEx ElPinceServo, ElCoudeServo, ElPoignetServo;
    private double ElPinceServoCurrentPosition, ElCoudeServoCurrentPosition,ElPoignetServoCurrentPosition;
    private boolean PoignetZero, PinceZero, CoudeZero, resetElevator;
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

        elevatorRightMotorEncoder = new MotorEx(globalSubsystem.hardwareMap, Constants.GrabberConstants.GRABBER_RIGHT_MOTOR_NAME);

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

        elevatorRightMotorEncoder.resetEncoder();

        elevatorMotors = new MotorGroup(elevatorLeftMotor, elevatorRightMotor);


        elevatorDownLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, Constants.ElevatorConstants.ELEVATOR_DOWN_LIMIT);
        //bottomLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, ArmConstants.BOTTOM_LIMIT_SWITCH_NAME);

        elevatorDownLimit.setMode(DigitalChannel.Mode.INPUT);
        PoignetZero = false;
        PinceZero = true;
        CoudeZero = true;

        resetElevator = false;

    }

    public void setElevator(double power) {

        if (power<0 && !elevatorDownLimit.getState()) {
            elevatorMotors.stopMotor();
        }else if (power > 0 && !elevatorDownLimit.getState()) {
            ElPinceServoCurrentPosition = PINCE_SERVO_MIN_POSITION;
            elevatorMotors.set(power);
            PinceZero = false;
        }
        else if (power <0 && elevatorRightMotorEncoder.getCurrentPosition() < 50){
            elevatorMotors.set(power/1.3);
            ElCoudeServoCurrentPosition = COUDE_SERVO_MIN_POSITION;
            CoudeZero = false;
        }else {
            elevatorMotors.set(power);
        }

    }
    public void stopElevator() {
        elevatorMotors.stopMotor();
    }


    public void setPoignet() {
        if (PoignetZero){
            ElPoignetServoCurrentPosition = POIGNET_SERVO_MAX_POSITION;
            PoignetZero = false;

        }else{
            ElPoignetServoCurrentPosition = POIGNET_SERVO_MIN_POSITION;
            PoignetZero = true;
        }
    }

    public void setPince() {
        if (!PinceZero){
            ElPinceServoCurrentPosition = PINCE_SERVO_MAX_POSITION;
            PinceZero = true;
        }else{
            ElPinceServoCurrentPosition = PINCE_SERVO_MIN_POSITION;
            PinceZero = false;
        }
    }

    public void setCoude() {
        if (!CoudeZero){
            ElCoudeServoCurrentPosition = COUDE_SERVO_MAX_POSITION;
            CoudeZero = true;
        }else{
            ElCoudeServoCurrentPosition = COUDE_SERVO_MIN_POSITION;
            CoudeZero = false;
        }
    }


    public void periodic() {
        //telemetry.addData("Elevator Left", elevatorLeftMotor.getCurrentPosition());
        telemetry.addData("Elevator Right", elevatorRightMotorEncoder.getCurrentPosition());
        //telemetry.addData("Pince", pinceServo.get );
        //pinceServo.setPower(0.3);

        ElPinceServo.setPosition(ElPinceServoCurrentPosition);
        ElCoudeServo.setPosition(ElCoudeServoCurrentPosition);
        ElPoignetServo.setPosition(ElPoignetServoCurrentPosition);


        if (!elevatorDownLimit.getState()) {
            elevatorMotors.stopMotor();
            elevatorRightMotor.resetEncoder();
            resetElevator = true;

        }

    }


    public boolean startClimb() {
        ElCoudeServoCurrentPosition = COUDE_SERVO_MIN_POSITION;
        // Grabber Goes to limit switch and poignet goes up
        if (elevatorDownLimit.getState()){
            elevatorMotors.set(-0.4);}
        //Elevator Goes up
        return true;
    }
}
