package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name = "base")
public class Teleop extends Robot {
    @Override
    public void start() {
        robotDrive.setDefaultCommand(new RunCommand(() -> {
            double movementSlowdown;
            if ( pilotController.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON) ) {
                movementSlowdown = 0.6;
            }else {
                movementSlowdown = 1;
            }

            robotDrive.drive(
                    pilotController.getLeftY()  * movementSlowdown,
                    pilotController.getLeftX()  * movementSlowdown,
                    pilotController.getRightX() * movementSlowdown,
                    true);

            //if (pilotController.wasJustPressed(GamepadKeys.Button.START)) {
            //    robotDrive.resetPose(new Pose2d().rotate(-Math.PI * 1.5));
            //}
        }, robotDrive));

        robotGrabber.setDefaultCommand(new RunCommand(() -> {


            if (pilotController.isDown(GamepadKeys.Button.X)) {
                robotGrabber.setGrabberMiddle();

            }else if (pilotController.wasJustReleased(GamepadKeys.Button.X) ) {
                robotGrabber.setPoignetPrinceSpeed();
            }

            if (pilotController.isDown(GamepadKeys.Button.RIGHT_BUMPER) || pilotController.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                robotGrabber.setGrabberPower(
                        (pilotController.isDown(GamepadKeys.Button.LEFT_BUMPER) ? -0.8 : 0) +
                                (pilotController.isDown(GamepadKeys.Button.RIGHT_BUMPER) ? 0.8 : 0)
                );
            } else {
                robotGrabber.stopGrabberPower();
            }

            if (pilotController.isDown(GamepadKeys.Button.Y)){
                robotGrabber.vomit();
            }/*else {
                robotGrabber.stopPince();
            }*/

        }, robotGrabber));

        robotElevator.setDefaultCommand(new RunCommand(()->{

            boolean movementUpElevator = false;
            boolean movementDownElevator = false;
            if (pilotController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0 || copilotController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >0) {
                movementUpElevator = true;
            } else if (pilotController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 || copilotController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0) {
                movementDownElevator = true;
            }

            if (movementUpElevator || movementDownElevator) {
                robotElevator.setElevator(
                        (movementUpElevator ? 1 : 0) +
                                (movementDownElevator ? -1 : 0)
                );
            } else {
                robotElevator.stopElevator();
            }

            if (pilotController.wasJustPressed(GamepadKeys.Button.B)) {
                 robotElevator.setCoude();
            }
            if (pilotController.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON) || copilotController.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                robotElevator.openPince();
            }else if(pilotController.wasJustReleased(GamepadKeys.Button.RIGHT_STICK_BUTTON) || copilotController.wasJustReleased(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                robotElevator.closePince();
            }
            if (pilotController.wasJustPressed(GamepadKeys.Button.A) || copilotController.wasJustPressed(GamepadKeys.Button.A)){
                robotElevator.addToCoude();
            }


        },robotElevator));

        // when button dpad_up is pressed
        // execute command
        AtomicBoolean climbStep1s1 = new AtomicBoolean(false);
        AtomicBoolean climbStep1s2 = new AtomicBoolean(false);

        AtomicBoolean climbStep1 = new AtomicBoolean(false);



        AtomicBoolean finishedClim = new AtomicBoolean(false);
        pilotController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenReleased(new RunCommand(
                        () -> {
                            if(!climbStep1.get()) {

                                climbStep1.set(true);
                                if (climbStep1s1.get() && climbStep1s2.get()) {
                                    robotDrive.startClimb(1);

                                    //robotGrabber.extendSlides(0.4);
                                    //robotGrabber.extendSlides(0.8);
                                } else {
                                    climbStep1s1.set(robotGrabber.startClimb());
                                    climbStep1s2.set(robotElevator.startClimb());
                                }
                            }
                                else{
                                robotDrive.startClimb(0);
                                climbStep1.set(false);

                            }
                        }, robotElevator, robotGrabber,robotDrive
                ));
            /*.andThen(
                        new RunCommand(
                                () -> {
                                    robotDrive.startClimb(0.8);
                                    robotGrabber.extendSlides(0.8);
                                },robotDrive, robotGrabber
                )));*/

        AtomicBoolean climbStep2s1 = new AtomicBoolean(false);
        AtomicBoolean climbStep2s2 = new AtomicBoolean(false);
        
        pilotController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new RunCommand(
                        () -> {
                            /*robotDrive.startClimb(0);
                            if (climbStep2s1.get() && climbStep2s2.get()) {
                                robotGrabber.extendSlides(-1);
                                }
                            else {*/
                            if(!climbStep2s1.get()) {
                                climbStep2s1.set(robotGrabber.extendSlides(0.4));
                            }
                            else if (!finishedClim.get() && climbStep2s1.get()){
                                robotGrabber.lastPush();
                                robotDrive.startClimb(-1);
                                robotElevator.startClimb();
                                finishedClim.set(true);
                                climbStep2s1.set(false);
                            }else if(finishedClim.get()){
                                robotGrabber.stopGrabberPower();
                                robotDrive.startClimb(0);
                                robotElevator.stopElevator();
                                finishedClim.set(false);

                            }


                                //climbStep2s2.set(robotElevator.startClimb());
                            //}
                        }, robotElevator, robotGrabber, robotDrive
                ));
        /*.andThen(
                        new RunCommand(
                                () -> {
                                    //robotDrive.startClimb(0.1);
                                    robotGrabber.extendSlides(-1);
                                },robotDrive, robotGrabber
                        )));*/
        /*pilotController.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(
                pilotController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed()
        )*/


    }
}
