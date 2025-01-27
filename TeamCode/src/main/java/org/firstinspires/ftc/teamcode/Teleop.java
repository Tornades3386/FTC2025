package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "base")
public class Teleop extends Robot {
    @Override
    public void start() {
        robotDrive.setDefaultCommand(new RunCommand(() -> {
            if ( pilotController.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON) ) {
                double movementSlowdown = 0.6;
            }else {
                double movementSlowdown = 1;
            }

            robotDrive.drive(
                    pilotController.getLeftY()  *0.8,
                    pilotController.getLeftX()  *0.8,
                    pilotController.getRightX() *0.8,
                    false);

            //if (pilotController.wasJustPressed(GamepadKeys.Button.START)) {
            //    robotDrive.resetPose(new Pose2d().rotate(-Math.PI * 1.5));
            //}
        }, robotDrive));

        robotGrabber.setDefaultCommand(new RunCommand(() -> {
            boolean movementForwardGrabber = false;
            boolean movementBackGrabber = false;
            if (pilotController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) {
                movementForwardGrabber = true;
            } else if (pilotController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
                movementBackGrabber = true;
            }

            if (pilotController.wasJustReleased(GamepadKeys.Button.X)) {
                robotGrabber.setPoignetPrinceSpeed(
                );
            } /*else if (pilotController.wasJustReleased(GamepadKeys.Button.X) || pilotController.wasJustReleased(GamepadKeys.Button.A)) {
            robotGrabber.stopPrinceSpeed();

        }*/

            /*if (pilotController.isDown(GamepadKeys.Button.Y) || pilotController.isDown(GamepadKeys.Button.B)) {
                robotGrabber.setPoignetSpeed(
                        (pilotController.isDown(GamepadKeys.Button.Y) ? 0.1 : 0)+
                                (pilotController.isDown(GamepadKeys.Button.B) ? -0.1: 0)
            );}/*else if (pilotController.wasJustReleased(GamepadKeys.Button.Y) || pilotController.wasJustReleased(GamepadKeys.Button.B)) {
            robotGrabber.stop();*/

            if (movementForwardGrabber || movementBackGrabber) {
                robotGrabber.setGrabberPower(
                        (movementBackGrabber ? -0.8 : 0) +
                                (movementForwardGrabber ? 0.8 : 0)
                );
            } else {
                robotGrabber.stopGrabberPower();
            }
        }, robotGrabber));

        robotElevator.setDefaultCommand(new RunCommand(()->{
            if (pilotController.isDown(GamepadKeys.Button.DPAD_UP) || pilotController.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                robotElevator.setElevator(
                        (pilotController.isDown(GamepadKeys.Button.DPAD_UP) ? 1 : 0) +
                                (pilotController.isDown(GamepadKeys.Button.DPAD_DOWN) ? -1 : 0)
                );
            } else if (pilotController.wasJustReleased(GamepadKeys.Button.DPAD_UP) || pilotController.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) {
                robotElevator.stopElevator();
            }
        },robotElevator));
    }
}
