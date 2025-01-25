package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drive9axisIMU;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GlobalSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrabberSubsystem;

public class Robot extends OpMode {
    static final GlobalSubsystem robotGlobal = GlobalSubsystem.getInstance();
    static final Drive9axisIMU robotDrive = Drive9axisIMU.getInstance();
    static final GrabberSubsystem robotGrabber = GrabberSubsystem.getInstance();
    static final ElevatorSubsystem robotElevator = ElevatorSubsystem.getInstance();
    static GamepadEx pilotController;
    static GamepadEx copilotController;

    public void init() {
        pilotController = new GamepadEx(gamepad1);
        copilotController = new GamepadEx(gamepad2);

        robotGlobal.init(telemetry, hardwareMap);
        robotDrive.init();
        robotGrabber.init();
        robotElevator.init();


        robotGlobal.telemetry.addData("Status", "Initialized");

        CommandScheduler.getInstance().registerSubsystem(robotGlobal, robotDrive, robotGrabber, robotElevator);
    }

    public void init_loop() {
        final double start = System.nanoTime();
        pilotController.readButtons();
        copilotController.readButtons();
        CommandScheduler.getInstance().run();
        robotGlobal.telemetry.addData("=== Loop time ===", (System.nanoTime() - start) / 1e6);
    }

    public void loop() {
        final double start = System.nanoTime();
        pilotController.readButtons();
        copilotController.readButtons();
        CommandScheduler.getInstance().run();
        robotGlobal.telemetry.addData("=== Loop time ===", (System.nanoTime() - start) / 1e6);
    }

    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}