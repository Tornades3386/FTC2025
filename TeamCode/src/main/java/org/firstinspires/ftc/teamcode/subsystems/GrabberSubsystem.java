package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.Constants.GrabberConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class GrabberSubsystem extends SubsystemBase {
    private static final GrabberSubsystem INSTANCE = new GrabberSubsystem();
    private MotorEx grabberRightMotor;
    private MotorEx grabberLeftMotor;
    private CRServo pinceServo;
    private ServoEx poignetServo;
    private Telemetry telemetry;
    private static double newPositionPINCE;
    private static double newPositionPoignet;
    private boolean dropState, pinceState, rightState;
    private DigitalChannel grabberDropLimit;

    private GrabberSubsystem() {}

    public static GrabberSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();
        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;

        telemetry = globalSubsystem.telemetry;

        grabberRightMotor = new MotorEx(globalSubsystem.hardwareMap, GrabberConstants.GRABBER_LEFT_MOTOR_NAME);
        grabberLeftMotor = new MotorEx(globalSubsystem.hardwareMap, GrabberConstants.GRABBER_RIGHT_MOTOR_NAME);

        poignetServo = new SimpleServo(hardwareMap, GrabberConstants.POIGNET_SERVO_NAME, 0.25, 1);
        pinceServo = hardwareMap.get(CRServo.class, GrabberConstants.PINCE_SERVO_NAME);


        grabberRightMotor.setRunMode(Motor.RunMode.RawPower);
        grabberRightMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        grabberLeftMotor.setRunMode(Motor.RunMode.RawPower);
        grabberLeftMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);




        grabberRightMotor.setInverted(false);
        grabberLeftMotor.setInverted(true);

        grabberRightMotor.resetEncoder();
        grabberLeftMotor.resetEncoder();

        //poignetServo.setPosition(0.50);
        //pinceServo.set(0.50);

        pinceState = false;
        rightState = false;
        dropState = false;


        //rotationEncoder = new AnalogEncoder(ArmConstants.ROTATION_ENCODER_NAME, 270);
        //extendEncoder = new AnalogEncoder(ArmConstants.EXTEND_ENCODER_NAME, 360);

        grabberDropLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, GrabberConstants.GRABBER_DROP_LIMIT);
        //bottomLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, ArmConstants.BOTTOM_LIMIT_SWITCH_NAME);

        grabberDropLimit.setMode(DigitalChannel.Mode.INPUT);
        //bottomLimit.setMode(DigitalChannel.Mode.INPUT);
    }


    /*public void setPrinceSpeed(double speed) {
        //newPositionPINCE = pinceServo.getPosition() + speed;
        //pinceServo.setPosition(pinceServo.getPosition() + speed);
        if (!pinceState){
            pinceServo.setPower(speed);
            pinceState = true;
        }
        else if (pinceState) {
            pinceServo.setPower(0);
            pinceState = false;
        }
    }*/

    public void stopPrinceSpeed() {pinceServo.setPower(0);}

    public void setPoignetPrinceSpeed() {
        poignetServo.setPosition(0.25);
        pinceServo.setPower(1);
    }

    public void setGrabberPower(double power){
        if (power < 0 ) {

            if (!grabberDropLimit.getState() & dropState) {

                poignetServo.setPosition(1);
                grabberRightMotor.set(0);
                grabberLeftMotor.set(0);
                while (dropState) {
                    grabberRightMotor.set(0);
                    grabberLeftMotor.set(0);

                    if (poignetServo.getPosition() == 1) {
                        pinceServo.setPower(-0.25);
                        dropState = false;
                    }

                }
            } else if (grabberRightMotor.getCurrentPosition() <= 1) {
                grabberRightMotor.set(0);
                grabberLeftMotor.set(0);
            }
            else  {
                grabberRightMotor.set(power);
                grabberLeftMotor.set(power);
            }
        }

        else {
            grabberRightMotor.set(power);
            grabberLeftMotor.set(power);
            dropState = true;

        }

    }

    public void stopGrabberPower(){
        grabberRightMotor.set(0);
        grabberLeftMotor.set(0);
    }



    public void periodic() {
        telemetry.addData("Poignet", poignetServo.getPosition());
        telemetry.addData("Grabber Encoder", grabberRightMotor.getCurrentPosition());
        //telemetry.addData("Pince", pinceServo.get );
        //pinceServo.setPower(0.3);


    }
}
