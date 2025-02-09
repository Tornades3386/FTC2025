/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.COUDE_SERVO_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.COUDE_SERVO_MIN_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.ELEVATOR_COUDE_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.ELEVATOR_PINCE_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.ELEVATOR_POIGNET_SERVO_NAME;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.PINCE_SERVO_CLOSED_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.PINCE_SERVO_OPEN_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.POIGNET_SERVO_MAX_POSITION;
import static org.firstinspires.ftc.teamcode.Constants.ElevatorConstants.POIGNET_SERVO_MIN_POSITION;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.GlobalSubsystem;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")

public class RobotAutoSlide extends LinearOpMode {

    /* Declare OpMode members. */
    private static MotorEx frontLeft, grabberRightMotor, grabberLeftMotor;
    private static MotorEx frontRight;
    private static MotorEx rearLeft, elevatorRightMotorEncoder;
    private static MotorEx rearRight,elevatorRightMotor, elevatorLeftMotor;

    private ElapsedTime     runtime = new ElapsedTime();


    private DigitalChannel  grabberLevelLimit;
    private double ElPinceServoCurrentPosition, ElCoudeServoCurrentPosition, ElPoignetServoCurrentPosition;

    static final double     FORWARD_SPEED = 0.7;
    static final double     TURN_SPEED    = 0.5;

    private ServoEx ElPinceServo, ElCoudeServo, ElPoignetServo;

    @Override
    public void runOpMode() {
//        GlobalSubsystem globalSubsystem = GlobalSubsystem.getInstance();
//        HardwareMap hardwareMap = GlobalSubsystem.getInstance().hardwareMap;/

        // Initialize the drive system variables.
        frontLeft = new MotorEx(hardwareMap, Constants.DriveConstants.FRONT_LEFT_MOTOR_NAME);
        frontRight = new MotorEx(hardwareMap, Constants.DriveConstants.FRONT_RIGHT_MOTOR_NAME);
        rearLeft = new MotorEx(hardwareMap, Constants.DriveConstants.REAR_LEFT_MOTOR_NAME);
        rearRight = new MotorEx(hardwareMap, Constants.DriveConstants.REAR_RIGHT_MOTOR_NAME);

        elevatorRightMotor = new MotorEx(hardwareMap, Constants.ElevatorConstants.ELEVATOR_RIGHT_MOTOR_NAME);
        elevatorLeftMotor = new MotorEx(hardwareMap, Constants.ElevatorConstants.ELEVATOR_LEFT_MOTOR_NAME);

        frontLeft.setInverted(true);
        rearLeft.setInverted(true);

        elevatorRightMotor.setRunMode(Motor.RunMode.RawPower);
        elevatorRightMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        elevatorLeftMotor.setRunMode(Motor.RunMode.RawPower);
        elevatorLeftMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        elevatorRightMotor.setInverted(false);
        elevatorLeftMotor.setInverted(true);



        grabberLevelLimit = hardwareMap.get(DigitalChannel.class, Constants.GrabberConstants.GRABBER_FLOOR_LIMIT);

        grabberLevelLimit.setMode(DigitalChannel.Mode.INPUT);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        rearLeft.setRunMode(Motor.RunMode.RawPower);
        rearRight.setRunMode(Motor.RunMode.RawPower);

        elevatorRightMotorEncoder = new MotorEx(hardwareMap, Constants.GrabberConstants.GRABBER_RIGHT_MOTOR_NAME);

//        grabberRightMotor.resetEncoder();

        frontLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);


        ElPinceServo = new SimpleServo(hardwareMap, ELEVATOR_PINCE_SERVO_NAME, PINCE_SERVO_CLOSED_POSITION, PINCE_SERVO_OPEN_POSITION);
        ElCoudeServo = new SimpleServo(hardwareMap, ELEVATOR_COUDE_SERVO_NAME, COUDE_SERVO_MIN_POSITION, COUDE_SERVO_MAX_POSITION);
        ElPoignetServo = new SimpleServo(hardwareMap, ELEVATOR_POIGNET_SERVO_NAME, POIGNET_SERVO_MIN_POSITION, POIGNET_SERVO_MAX_POSITION);

        ElPinceServo.setPosition(PINCE_SERVO_CLOSED_POSITION);
        ElCoudeServo.setPosition(COUDE_SERVO_MIN_POSITION);
        ElPoignetServo.setPosition(POIGNET_SERVO_MAX_POSITION);


        elevatorRightMotorEncoder.resetEncoder();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        //advance just a bit
        frontLeft.set(FORWARD_SPEED);
        frontRight.set(FORWARD_SPEED);
        rearLeft.set(FORWARD_SPEED);
        rearRight.set(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //advance just a bit
        frontLeft.set(FORWARD_SPEED);
        frontRight.set(-FORWARD_SPEED);
        rearLeft.set(-FORWARD_SPEED);
        rearRight.set(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5.0)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.
        //Back 0.3 sec
        /*frontLeft.set(-FORWARD_SPEED);
        frontRight.set(-FORWARD_SPEED);
        rearLeft.set(-FORWARD_SPEED);
        rearRight.set(-FORWARD_SPEED);

        grabberLeftMotor.set(0.7);
        grabberRightMotor.set(0.7);

        runtime.reset();
        while (opModeIsActive() && ((!grabberLevelLimit.getState() || grabberRightMotor.getCurrentPosition() <500 ))) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //advance just a bit
        frontLeft.set(FORWARD_SPEED);
        frontRight.set(FORWARD_SPEED);
        rearLeft.set(FORWARD_SPEED);
        rearRight.set(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 1:  Turn for 1 seconds
        frontLeft.set(-FORWARD_SPEED);
        frontRight.set(FORWARD_SPEED);
        rearLeft.set(-FORWARD_SPEED);
        rearRight.set(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Raise elevator
        elevatorLeftMotor.set(FORWARD_SPEED);
        elevatorRightMotor.set(FORWARD_SPEED);
        ElCoudeServo.setPosition(COUDE_SERVO_MAX_POSITION);
        runtime.reset();
        while (opModeIsActive() && (elevatorRightMotorEncoder.getCurrentPosition() < 500)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  drop sample
        ElPoignetServo.setPosition(POIGNET_SERVO_MAX_POSITION);
        frontLeft.set(FORWARD_SPEED);
        frontRight.set(FORWARD_SPEED);
        rearLeft.set(FORWARD_SPEED);
        rearRight.set(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop
        frontLeft.set(0);
        frontRight.set(0);
        rearLeft.set(0);
        rearRight.set(0);

        ElPinceServo.setPosition(PINCE_SERVO_OPEN_POSITION);*/

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
