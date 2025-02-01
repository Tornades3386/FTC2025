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
                    true);

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

            if (pilotController.isDown(GamepadKeys.Button.X)) {
                robotGrabber.setGrabberMiddle();

            }else if (pilotController.wasJustReleased(GamepadKeys.Button.X) ) {
                robotGrabber.setPoignetPrinceSpeed();
            }

            if (movementForwardGrabber || movementBackGrabber) {
                robotGrabber.setGrabberPower(
                        (movementBackGrabber ? -0.8 : 0) +
                                (movementForwardGrabber ? 0.8 : 0)
                );
            } else {
                robotGrabber.stopGrabberPower();
            }

            if (pilotController.isDown(GamepadKeys.Button.Y)){
                robotGrabber.vomit();
            }
            if (pilotController.wasJustReleased(GamepadKeys.Button.DPAD_UP)){
                robotGrabber.startClimb();
            }
        }, robotGrabber));

        robotElevator.setDefaultCommand(new RunCommand(()->{
            if (pilotController.isDown(GamepadKeys.Button.RIGHT_BUMPER) || pilotController.isDown(GamepadKeys.Button.LEFT_BUMPER) || copilotController.isDown(GamepadKeys.Button.LEFT_BUMPER) || copilotController.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                robotElevator.setElevator(
                        (pilotController.isDown(GamepadKeys.Button.RIGHT_BUMPER) ? 1 : 0) +
                                (pilotController.isDown(GamepadKeys.Button.LEFT_BUMPER) ? -1 : 0)
                );
            } else if (pilotController.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER) || pilotController.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER) || copilotController.isDown(GamepadKeys.Button.LEFT_BUMPER) || copilotController.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                robotElevator.stopElevator();
            }
            if (pilotController.isDown(GamepadKeys.Button.X)) {
                 robotElevator.setPoignet();
            }
            if (pilotController.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON) || copilotController.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                robotElevator.setPince();
            }
            if (pilotController.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON) || copilotController.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                robotElevator.setCoude();
            }
            if (pilotController.wasJustReleased(GamepadKeys.Button.DPAD_UP)){
                robotElevator.startClimb();
            }

        },robotElevator));
    }
}
