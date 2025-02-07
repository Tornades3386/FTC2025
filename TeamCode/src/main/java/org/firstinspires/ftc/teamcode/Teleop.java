package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.subsystems.GlobalSubsystem;

import java.util.concurrent.atomic.AtomicBoolean;

@TeleOp(name = "base")
public class Teleop extends Robot {
    @Override
    public void start() {
        robotDrive.setDefaultCommand(new RunCommand(() -> {
            double movementSlowdown;
            if (pilotController.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                movementSlowdown = 0.6;
            } else {
                movementSlowdown = 1;
            }

            robotDrive.drive(
                pilotController.getLeftY() * movementSlowdown,
                pilotController.getLeftX() * movementSlowdown,
                pilotController.getRightX() * movementSlowdown,
                true);

            //if (pilotController.wasJustPressed(GamepadKeys.Button.START)) {
            //    robotDrive.resetPose(new Pose2d().rotate(-Math.PI * 1.5));
            //}
        }, robotDrive));

        robotGrabber.setDefaultCommand(new RunCommand(() -> {
            if (pilotController.isDown(GamepadKeys.Button.X)) {
                robotGrabber.setGrabberMiddle();
            } else if (pilotController.wasJustReleased(GamepadKeys.Button.X)) {
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

            if (pilotController.isDown(GamepadKeys.Button.Y)) {
                robotGrabber.vomit();
            } else if (pilotController.wasJustReleased(GamepadKeys.Button.Y)) {
                robotGrabber.stopPince();
            }
        }, robotGrabber));

        robotElevator.setDefaultCommand(new RunCommand(() -> {
            boolean movementUpElevator = false;
            boolean movementDownElevator = false;
            if (pilotController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0 || copilotController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0) {
                movementUpElevator = true;
            } else if (pilotController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0 || copilotController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0) {
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
            if (pilotController.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON) || copilotController.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                robotElevator.openPince();
            } else if (pilotController.wasJustReleased(GamepadKeys.Button.RIGHT_STICK_BUTTON) || copilotController.wasJustReleased(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
                robotElevator.closePince();
            }
            if (pilotController.wasJustPressed(GamepadKeys.Button.A) || copilotController.wasJustPressed(GamepadKeys.Button.A)) {
                robotElevator.addToCoude();
            }
        }, robotElevator));

        // when button dpad_up is pressed
        // execute command
        AtomicBoolean climbStep1s1 = new AtomicBoolean(false);
        AtomicBoolean climbStep1s2 = new AtomicBoolean(false);

        AtomicBoolean climbStep1 = new AtomicBoolean(false);

//        pilotController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .toggleWhenPressed(new RunCommand(
//                        () -> {
//                            if(!climbStep1.get()) {
//
//                                climbStep1.set(true);
//                                if (climbStep1s1.get() && climbStep1s2.get()) {
//                                    robotDrive.startClimb(1);
//
//                                    //robotGrabber.extendSlides(0.4);
//                                    //robotGrabber.extendSlides(0.8);
//                                } else {
//                                    climbStep1s1.set(robotGrabber.startClimb());
//                                    climbStep1s2.set(robotElevator.startClimb());
//                                }
//                            }
//                                else{
//                                robotDrive.startClimb(0);
//                                climbStep1.set(false);
//
//                            }
//                        }, robotElevator, robotGrabber,robotDrive
//                ));

        Command climbCommand = new FunctionalCommand(
            () -> {},
            () -> {},
            bool -> robotDrive.startClimb(1),
            () -> {
                boolean a = robotGrabber.startClimb();
                boolean b = robotElevator.startClimb();
                GlobalSubsystem.getInstance().telemetry.addLine("Running climb 1");
                GlobalSubsystem.getInstance().telemetry.addData("Grabber state", a);
                GlobalSubsystem.getInstance().telemetry.addData("Elevator state", b);
                return (a && b) || pilotController.isDown(GamepadKeys.Button.BACK);
            }
        );
        pilotController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .toggleWhenPressed(new StartEndCommand(
                () -> CommandScheduler.getInstance().schedule(climbCommand),
                () -> {
                    robotGrabber.stopGrabberPower();
                    robotDrive.startClimb(0);
                    robotElevator.stopElevator();
                }
            ));

        pilotController.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenHeld(new StartEndCommand(
                () -> robotGrabber.extendSlides(0.4),
                () -> robotGrabber.extendSlides(0)
            ));

        pilotController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .toggleWhenPressed(new StartEndCommand(
                () -> {
                    robotGrabber.lastPush();
                    robotDrive.startClimb(-1);
                    robotElevator.startClimb();
                },
                () -> {
                    robotGrabber.stopGrabberPower();
                    robotDrive.startClimb(0);
                    robotElevator.stopElevator();
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
