package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
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
            }else {
                robotGrabber.stopPince();
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
            if (pilotController.wasJustReleased(GamepadKeys.Button.B)) {
                 robotElevator.setPoignet();
            }
            if (pilotController.wasJustReleased(GamepadKeys.Button.RIGHT_STICK_BUTTON) || copilotController.wasJustReleased(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                robotElevator.setPince();
            }
            if (pilotController.wasJustReleased(GamepadKeys.Button.A) || copilotController.wasJustReleased(GamepadKeys.Button.A)){
                robotElevator.setCoude();
            }


        },robotElevator));

        // when button dpad_up is pressed
        // execute command
        pilotController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenReleased(new RunCommand(
                        () -> {
                            robotGrabber.startClimb();
                            robotElevator.startClimb();
                        }, robotElevator, robotGrabber
                ).andThen());

        // when both up and down are pressed (example)
        //
        /*pilotController.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(
                pilotController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed()
        )*/

        // btw maybe not RunCommand since it runs infinitely
        // you can use the InstantCommand for a thing that runs once
        // got it?
        // yup
        // welp gl on this friendly competition
        // hopefully i'd have time to help next week before feb 8
        // <3 good luck in school
        // tyyyyy
        // bye
    }
}
