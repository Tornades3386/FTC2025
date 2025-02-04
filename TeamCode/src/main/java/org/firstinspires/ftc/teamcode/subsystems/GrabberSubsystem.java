package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.GrabberConstants.POIGNET_SERVO_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.GrabberConstants.POIGNET_SERVO_MID_POSITION;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants.GrabberConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class GrabberSubsystem extends SubsystemBase {
    private static final GrabberSubsystem INSTANCE = new GrabberSubsystem();
    private MotorEx grabberRightMotor;
    private MotorEx grabberLeftMotor;
    private CRServo pinceServo;
    private ServoEx poignetServo;
    private Telemetry telemetry;

    private static double targetPositionPoignet, currentPosition;
    private boolean dropState, pinceState, rightState, startClimb, resetGrabber;
    private DigitalChannel grabberDropLimit;
    private ColorSensor colorPince;
    private MotorGroup slideMotors;
    private DistanceSensor colorPince_DistanceSensor;

    private GrabberSubsystem() {}

    public static GrabberSubsystem getInstance() {
        return INSTANCE;
    }

    public void init() {
        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();
        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;

        telemetry = globalSubsystem.telemetry;

        grabberRightMotor = new MotorEx(globalSubsystem.hardwareMap, GrabberConstants.GRABBER_RIGHT_MOTOR_NAME);
        grabberLeftMotor = new MotorEx(globalSubsystem.hardwareMap, GrabberConstants.GRABBER_LEFT_MOTOR_NAME);

        poignetServo = new SimpleServo(hardwareMap, GrabberConstants.POIGNET_SERVO_NAME, 0.25, 1);
        pinceServo = hardwareMap.get(CRServo.class, GrabberConstants.PINCE_SERVO_NAME);



        grabberRightMotor.setRunMode(Motor.RunMode.RawPower);
        grabberRightMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        grabberLeftMotor.setRunMode(Motor.RunMode.RawPower);
        grabberLeftMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);



        grabberRightMotor.setInverted(false);
        grabberLeftMotor.setInverted(true);

        grabberRightMotor.resetEncoder();


        slideMotors = new MotorGroup(grabberLeftMotor, grabberRightMotor);

        currentPosition = 0.50;
        poignetServo.setPosition(currentPosition);
        pinceServo.setPower(0);

        startClimb = false;

        resetGrabber = false;

        pinceState = false;
        rightState = false;
        dropState = false;

        grabberDropLimit = globalSubsystem.hardwareMap.get(DigitalChannel.class, GrabberConstants.GRABBER_DROP_LIMIT);

        grabberDropLimit.setMode(DigitalChannel.Mode.INPUT);

        colorPince = hardwareMap.get(ColorSensor.class, "colorPince");
        colorPince_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorPince");
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

    public void vomit(){pinceServo.setPower(-1);}
    public void setPoignetPrinceSpeed() {
        if (!pinceState){
            //Descendre la pince et tourner
            currentPosition = Math.max(GrabberConstants.POIGNET_SERVO_MIN_POSITION, Math.min(POIGNET_SERVO_MAX_POSITION, poignetServo.getPosition() - 2));
            //poignetServo.setPosition(Math.max(0.25, Math.min(1, poignetServo.getPosition() - 2)));
            pinceServo.setPower(1);
            pinceState = true;
        }else {//if (grabberRightMotor.getCurrentPosition() > 400) {
            currentPosition = Math.max(GrabberConstants.POIGNET_SERVO_MIN_POSITION, Math.min(POIGNET_SERVO_MAX_POSITION, poignetServo.getPosition() + 2));
            //poignetServo.setPosition(Math.max(0.25, Math.min(1, poignetServo.getPosition() + 2)));
            pinceServo.setPower(0);
            pinceState = false;
        }
    }

    public void setGrabberMiddle(){
        currentPosition = POIGNET_SERVO_MID_POSITION;
    }

    public void setGrabberPower(double power){
        //Quand on rentre le Grabber
        if (power < 0  ) {
            //newPositionPoignet = Math.max(0.25, Math.min(1, poignetServo.getPosition() + 2));
            //poignetServo.setPosition(newPositionPoignet);
            //Si on a un sample
            if (Double.parseDouble(JavaUtil.formatNumber(colorPince_DistanceSensor.getDistance(DistanceUnit.CM), 3)) < 3.2){
                //currentPosition = Math.max(GrabberConstants.POIGNET_SERVO_MIN_POSITION, Math.min(GrabberConstants.POIGNET_SERVO_MAX_POSITION, poignetServo.getPosition() + 2));
                currentPosition = POIGNET_SERVO_MID_POSITION;
            }
            //Quand on arrive a ;a limit switch
            if (!grabberDropLimit.getState() ){
                //Si on a un smaple
                if (Double.parseDouble(JavaUtil.formatNumber(colorPince_DistanceSensor.getDistance(DistanceUnit.CM), 3)) < 3.2) {
                    slideMotors.stopMotor();
                    pinceServo.setPower(0.4);
                    final double TOLERANCE = 0.1;
                    //currentPosition = poignetServo.getPosition();
                    currentPosition = poignetServo.getPosition();
                    if (poignetServo.getPosition() <= 0.99) {
                        currentPosition += 0.02;
                    } else {
                        pinceServo.setPower(-0.3);
                    }
                }
                //Si on n'a pas de sample
                if (Double.parseDouble(JavaUtil.formatNumber(colorPince_DistanceSensor.getDistance(DistanceUnit.CM), 3)) > 3) {

                    slideMotors.set(power);

                    //pinceState = false;

                    currentPosition = (Math.max(GrabberConstants.POIGNET_SERVO_MIN_POSITION, Math.min(POIGNET_SERVO_MAX_POSITION, poignetServo.getPosition() - 2)));
                    //currentPosition = poignetServo.getPosition();
                    pinceServo.setPower(0);

                }



            } else if (grabberLeftMotor.getCurrentPosition() <= -250 && resetGrabber) {
                slideMotors.stopMotor();

            }
            else {
                slideMotors.set(power);
            }

        }

        else if (power > 0 && grabberLeftMotor.getCurrentPosition() <2700 ) {
            slideMotors.set(power);



        }
        else {
            slideMotors.stopMotor();
        }

    }

    public void stopGrabberPower(){
        slideMotors.stopMotor();
    }

    public boolean startClimb(){
        currentPosition = Math.max(GrabberConstants.POIGNET_SERVO_MIN_POSITION, Math.min(POIGNET_SERVO_MAX_POSITION, poignetServo.getPosition() + 2));
        if (grabberLeftMotor.getCurrentPosition() < -40 || grabberLeftMotor.getCurrentPosition() >50) {
            if (grabberLeftMotor.getCurrentPosition() > 10) {
                slideMotors.set(-0.4);
            } else if (grabberLeftMotor.getCurrentPosition() < 10) {
                slideMotors.set(0.4);
            }
        }
        else{
            slideMotors.stopMotor();
            return true;
        }

    }
    public void stopPince() {
        pinceServo.setPower(0);
    }

    public void extendSlides(double power) {
        if (power > 0 && grabberLeftMotor.getCurrentPosition() < 1000){
            slideMotors.set(power);
        }else{
            slideMotors.stopMotor();
            
        }
    }



    public void periodic() {
        telemetry.addData("Poignet", poignetServo.getPosition());
        telemetry.addData("Grabber Encoder", grabberLeftMotor.getCurrentPosition());
        telemetry.addData("Grabber Limit Switch", grabberDropLimit.getState());
        telemetry.addData("Distance (cm)", Double.parseDouble(JavaUtil.formatNumber(colorPince_DistanceSensor.getDistance(DistanceUnit.CM), 3)));

        if(Double.parseDouble(JavaUtil.formatNumber(colorPince_DistanceSensor.getDistance(DistanceUnit.CM),3))<2 ) {

            if (pinceServo.getPower() > 0) {
                pinceServo.setPower(0.3);
                pinceState = false;
            }
            ;//telemetry.addData("Pince", pinceServo.get );
        }
        poignetServo.setPosition(currentPosition);

        if (!grabberDropLimit.getState()){
            grabberLeftMotor.resetEncoder();
            resetGrabber = true;
        }
        if(grabberLeftMotor.getCurrentPosition() < -50){
            currentPosition = POIGNET_SERVO_MID_POSITION;
        }
        //pinceServo.setPower(0.3);
        if (startClimb){
            if (grabberLeftMotor.getCurrentPosition()> 0) {
                slideMotors.set(-0.5);
            }else if (grabberLeftMotor.getCurrentPosition()< 0)
                slideMotors.set(0.5);
            else{
                slideMotors.stopMotor();
                currentPosition = POIGNET_SERVO_MAX_POSITION;
            }
        }

    }


}
