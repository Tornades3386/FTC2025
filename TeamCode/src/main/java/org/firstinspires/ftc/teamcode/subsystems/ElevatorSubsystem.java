package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.COUDE_SERVO_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.COUDE_SERVO_MID_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.COUDE_SERVO_MIN_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.ELEVATOR_COUDE_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.ELEVATOR_PINCE_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.ELEVATOR_POIGNET_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.PINCE_SERVO_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.PINCE_SERVO_CLOSED_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.POIGNET_SERVO_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.POIGNET_SERVO_MIN_POSITION;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
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
    private double ElPinceServoCurrentPosition, ElCoudeServoCurrentPosition, ElPoignetServoCurrentPosition;
    private boolean PoignetZero, CoudeZero, resetElevator, FinishClimb, enTransitCoude, enTransitPince, pinceClose;
    private MotorGroup elevatorMotors;

    private ElevatorSubsystem() {
    }

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

        ElPinceServo = new SimpleServo(hardwareMap, ELEVATOR_PINCE_SERVO_NAME, PINCE_SERVO_CLOSED_POSITION, PINCE_SERVO_OPEN_POSITION);
        ElCoudeServo = new SimpleServo(hardwareMap, ELEVATOR_COUDE_SERVO_NAME, COUDE_SERVO_MIN_POSITION, COUDE_SERVO_MAX_POSITION);
        ElPoignetServo = new SimpleServo(hardwareMap, ELEVATOR_POIGNET_SERVO_NAME, POIGNET_SERVO_MIN_POSITION, POIGNET_SERVO_MAX_POSITION);

        ElPinceServoCurrentPosition = PINCE_SERVO_CLOSED_POSITION;
        ElCoudeServoCurrentPosition = COUDE_SERVO_MIN_POSITION;
        ElPoignetServoCurrentPosition = POIGNET_SERVO_MAX_POSITION;

        ElPinceServo.setPosition(ElPinceServoCurrentPosition);
        ElCoudeServo.setPosition(ElCoudeServoCurrentPosition);
        ElPoignetServo.setPosition(ElPoignetServoCurrentPosition);


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
        PoignetZero = true;
        CoudeZero = false;
        pinceClose = false;

        enTransitCoude = true;

        enTransitPince = false;

        resetElevator = false;

    }

    public void setElevator(double power) {

        if (power < 0 && !elevatorDownLimit.getState()) {
            elevatorMotors.stopMotor();
            //doesnt set le poignet
            ElPoignetServoCurrentPosition = POIGNET_SERVO_MAX_POSITION;
            ElPinceServoCurrentPosition = PINCE_SERVO_OPEN_POSITION;
        } else if (power > 0 && !elevatorDownLimit.getState()) {
            if (!pinceClose) {
                enTransitPince = true;
            } else if (!enTransitPince && pinceClose) {
                pinceClose=false;
                elevatorMotors.set(power);
            }
            //ElPinceServoCurrentPosition = PINCE_SERVO_MIN_POSITION

        } else {
            elevatorMotors.set(power);
        }

    }

    public void stopElevator() {
        elevatorMotors.stopMotor();
    }


    /*public void setPoignet() {
        if (PoignetZero){
            ElPoignetServoCurrentPosition = ElPoignetServoCurrentPosition + ;
            PoignetZero = false;

        }else{
            ElPoignetServoCurrentPosition = POIGNET_SERVO_MIN_POSITION;
            PoignetZero = true;
        }
    }*/

    public void openPince() {

        ElPinceServoCurrentPosition = PINCE_SERVO_OPEN_POSITION;

    }

    public void closePince() {

        ElPinceServoCurrentPosition = PINCE_SERVO_CLOSED_POSITION;

    }

    public void setCoude() {

        enTransitCoude = true;
        /*if (!enTransitCoude){
            ElCoudeServoCurrentPosition = COUDE_SERVO_MIN_POSITION;
            enTransitCoude = true;
        }else{
            ElCoudeServoCurrentPosition = COUDE_SERVO_MAX_POSITION;
            enTransitCoude = false;
        }*/
    }

    public void addToCoude() {
        if (ElCoudeServoCurrentPosition >= 0.70) {
            ElCoudeServoCurrentPosition = COUDE_SERVO_MAX_POSITION;
        }
    }

    public boolean startClimb() {
        if (!FinishClimb) {
            ElCoudeServoCurrentPosition = COUDE_SERVO_MIN_POSITION;
            // Grabber Goes to limit switch and poignet goes up
            if (elevatorDownLimit.getState()) {
                elevatorMotors.set(-1);
                FinishClimb = false;
                return false;
            } else {
                FinishClimb = true;
                return true;
            }
        } else {
            return true;
        }
        //Elevator Goes up
    }


    public void periodic() {
        //telemetry.addData("Elevator Left", elevatorLeftMotor.getCurrentPosition());
        telemetry.addData("Elevator Right", elevatorRightMotorEncoder.getCurrentPosition());
        telemetry.addData("Elevator Limit Switch", elevatorDownLimit.getState());
        telemetry.addData("EN TRANSIT PINCE", enTransitPince);
        telemetry.addData("EN TRANSIT COUDE", enTransitCoude);
        telemetry.addData("CLIMB ELEVATOR", FinishClimb);
        telemetry.addData("PINCE ELEVATOR", ElPinceServo.getPosition());
        telemetry.addData("Coude position", ElCoudeServo.getPosition());
        //telemetry.addData("Pince", pinceServo.get );
        //pinceServo.setPower(0.3);

        ElPinceServo.setPosition(ElPinceServoCurrentPosition);
        ElCoudeServo.setPosition(ElCoudeServoCurrentPosition);
        ElPoignetServo.setPosition(ElPoignetServoCurrentPosition);


        if (!elevatorDownLimit.getState()) {
            elevatorMotors.stopMotor();
            elevatorRightMotor.resetEncoder();
            ElPoignetServoCurrentPosition = POIGNET_SERVO_MAX_POSITION;
            resetElevator = true;
        }

        if (enTransitPince) {
            if (ElPinceServoCurrentPosition > PINCE_SERVO_CLOSED_POSITION) {
                ElPinceServoCurrentPosition -= 0.04;
            } else {
                pinceClose = true;
                enTransitPince = false;
            }
        }

        if (enTransitCoude) {
            if (!CoudeZero) {
                if (ElCoudeServoCurrentPosition < COUDE_SERVO_MID_POSITION) {
                    ElPoignetServoCurrentPosition = POIGNET_SERVO_MIN_POSITION;
                    ElCoudeServoCurrentPosition += 0.04;

                } else {
                    ElPoignetServoCurrentPosition = POIGNET_SERVO_MAX_POSITION;
                    CoudeZero = true;
                    enTransitCoude = false;
                }


            } else {
                if (ElCoudeServoCurrentPosition > COUDE_SERVO_MIN_POSITION) {
                    ElPoignetServoCurrentPosition = POIGNET_SERVO_MIN_POSITION;
                    ElCoudeServoCurrentPosition -= 0.03;

                } else {
                    ElPoignetServoCurrentPosition = POIGNET_SERVO_MAX_POSITION;
                    CoudeZero = false;
                    enTransitCoude = false;
                }
            }
        }

    }
}
