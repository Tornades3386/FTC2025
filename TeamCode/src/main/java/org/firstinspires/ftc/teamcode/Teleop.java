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
            double movementSlowdown = 1 - pilotController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.6;
            movementSlowdown *= 1 - pilotController.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) * 0.4;

            robotDrive.drive(new ChassisSpeeds(
                    pilotController.getLeftX() * movementSlowdown * Constants.DriveConstants.MAX_MOVEMENT_PER_SECOND,
                    pilotController.getLeftY() * movementSlowdown * Constants.DriveConstants.MAX_MOVEMENT_PER_SECOND,
                    pilotController.getRightX() * movementSlowdown * Constants.DriveConstants.MAX_ROTATION_PER_SECOND
            ), true, robotGlobal.driverRotation);

            //if (pilotController.wasJustPressed(GamepadKeys.Button.START)) {
            //    robotDrive.resetPose(new Pose2d().rotate(-Math.PI * 1.5));
            //}
        }, robotDrive));

        robotGrabber.setDefaultCommand(new RunCommand(() -> {
            if (pilotController.isDown(GamepadKeys.Button.X) || copilotController.isDown(GamepadKeys.Button.A)) {
                robotGrabber.setPrinceSpeed(
                        (pilotController.isDown(GamepadKeys.Button.X) ? 1 : 0) +
                                (pilotController.isDown(GamepadKeys.Button.A) ? -1 : 0)
                );
            } /*else if (copilotController.wasJustReleased(GamepadKeys.Button.X) || copilotController.wasJustReleased(GamepadKeys.Button.A)) {
            robotGrabber.stopPrinceSpeed();

        }*/

        /*if (pilotController.isDown(GamepadKeys.Button.Y) || copilotController.isDown(GamepadKeys.Button.B)) {
            robotGrabber.setPoignetSpeed(
                    (pilotController.isDown(GamepadKeys.Button.Y) ? 0.3 : 0)+
                            (pilotController.isDown(GamepadKeys.Button.B) ? -0.3 : 0)
            );}*/

            if (pilotController.isDown(GamepadKeys.Button.Y) || copilotController.isDown(GamepadKeys.Button.B)) {
                robotGrabber.setGrabberPower(
                        (pilotController.isDown(GamepadKeys.Button.Y) ? 0.1 : 0) +
                                (pilotController.isDown(GamepadKeys.Button.B) ? -0.1 : 0)
                );
            } else if (copilotController.wasJustReleased(GamepadKeys.Button.Y) || copilotController.wasJustReleased(GamepadKeys.Button.B)) {
                robotGrabber.stopGRabberPower();
            }
        }, robotGrabber));
    }
}
