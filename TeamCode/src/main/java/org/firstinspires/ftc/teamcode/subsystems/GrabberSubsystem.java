package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
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
    private boolean dropState, pinceState, rightState;
    private DigitalChannel grabberDropLimit;
    private ColorSensor colorPince;
    private DistanceSensor colorPince_DistanceSensor;

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


        grabberRightMotor.setRunMode(Motor.RunMode.PositionControl);
        grabberRightMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        grabberLeftMotor.setRunMode(Motor.RunMode.PositionControl);
        grabberLeftMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);




        grabberRightMotor.setInverted(false);
        grabberLeftMotor.setInverted(true);

        grabberRightMotor.resetEncoder();
        grabberLeftMotor.resetEncoder();

        currentPosition = 0.50;
        poignetServo.setPosition(currentPosition);
        pinceServo.setPower(0);

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

    public void setPoignetPrinceSpeed() {
        if (!pinceState){
            currentPosition = Math.max(0.25, Math.min(1, poignetServo.getPosition() - 2));
            //poignetServo.setPosition(Math.max(0.25, Math.min(1, poignetServo.getPosition() - 2)));
            pinceServo.setPower(1);
            pinceState = true;
        }else {//if (grabberRightMotor.getCurrentPosition() > 400) {
            currentPosition = Math.max(0.25, Math.min(1, poignetServo.getPosition() + 2));
            //poignetServo.setPosition(Math.max(0.25, Math.min(1, poignetServo.getPosition() + 2)));
            pinceServo.setPower(0);
            pinceState = false;
        }
    }

    public void setGrabberPower(double power){
        if (power < 0 ) {
            //newPositionPoignet = Math.max(0.25, Math.min(1, poignetServo.getPosition() + 2));
            //poignetServo.setPosition(newPositionPoignet);
            if (Double.parseDouble(JavaUtil.formatNumber(colorPince_DistanceSensor.getDistance(DistanceUnit.CM), 3)) < 3.2){
                currentPosition = Math.max(0.25, Math.min(1, poignetServo.getPosition() + 2));
            }
            if (!grabberDropLimit.getState()){
                if (Double.parseDouble(JavaUtil.formatNumber(colorPince_DistanceSensor.getDistance(DistanceUnit.CM), 3)) < 3.2) {
                    grabberRightMotor.stopMotor();
                    grabberLeftMotor.stopMotor();
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
                if (Double.parseDouble(JavaUtil.formatNumber(colorPince_DistanceSensor.getDistance(DistanceUnit.CM), 3)) > 3) {

                    grabberRightMotor.set(power);
                    grabberLeftMotor.set(power);
                    //pinceState = false;
                    currentPosition = (Math.max(0.25, Math.min(1, poignetServo.getPosition() - 2)));
                    pinceServo.setPower(0);

                }


                    /*for (; currentPosition < targetPositionPoignet;) {
                        poignetServo.setPosition(currentPosition);  // Update the servo to the current position
                        currentPosition += 0.001;  // Increment the position by 0.2
                        if (poignetServo.getPosition() > 0.98) {
                            //currentPosition = targetPositionPoignet;  // Prevent overshooting the target position
                            pinceServo.setPower(-0.45);
                            if (Double.parseDouble(JavaUtil.formatNumber(colorPince_DistanceSensor.getDistance(DistanceUnit.CM), 3)) > 3) {
                                poignetServo.setPosition(Math.max(0.25, Math.min(1, poignetServo.getPosition() - 2)));
                                pinceServo.setPower(0);
                                pinceState = false;
                            }
                        }
                    }*/
                    //poignetServo.setPosition(Math.max(0.25, Math.min(1, poignetServo.getPosition() + 2)));


                    /*if (poignetServo.getPosition() >=0.97){
                        pinceServo.setPower(-0.45);
                        if (Double.parseDouble(JavaUtil.formatNumber(colorPince_DistanceSensor.getDistance(DistanceUnit.CM), 3)) > 3) {
                            poignetServo.setPosition(Math.max(0.25, Math.min(1, poignetServo.getPosition() - 2)));
                            pinceServo.setPower(0);
                            pinceState = false;
                        }
                    }*/


                //pinceServo.setPower(-0.25);
                //dropState = false;
                //while (dropState && Math.abs(poignetServo.getPosition() - newPositionPoignet) > TOLERANCE) {
                    //grabberRightMotor.stopMotor();
                    //grabberLeftMotor.stopMotor();

                    //if (Math.abs(poignetServo.getPosition() - newPositionPoignet) < TOLERANCE){
                //pinceServo.setPower(-0.25);

                    //}

                //}
            } else if (grabberRightMotor.getCurrentPosition() <= 50) {
                grabberRightMotor.stopMotor();
                grabberLeftMotor.stopMotor();
            }
            else {
                grabberRightMotor.set(power);
                grabberLeftMotor.set(power);
            }

        }

        else if (power > 0 && grabberRightMotor.getCurrentPosition() <3100 ) {
            grabberRightMotor.set(power);
            grabberLeftMotor.set(power);

        }
        else {
            grabberRightMotor.stopMotor();
            grabberLeftMotor.stopMotor();
        }

    }

    public void stopGrabberPower(){
        grabberRightMotor.set(0);
        grabberLeftMotor.set(0);
    }



    public void periodic() {
        telemetry.addData("Poignet", poignetServo.getPosition());
        telemetry.addData("Grabber Encoder", grabberRightMotor.getCurrentPosition());
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
        /*if(grabberRightMotor.getCurrentPosition() <600){
            poignetServo.setPosition(Math.max(0.25, Math.min(1, poignetServo.getPosition() - 2)));
        }*/
        //pinceServo.setPower(0.3);


    }
}
