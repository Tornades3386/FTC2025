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

            if (pilotController.wasJustPressed(GamepadKeys.Button.START)) {
                robotDrive.resetPose(new Pose2d().rotate(-Math.PI * 1.5));
            }
        }, robotDrive));
    }
}
