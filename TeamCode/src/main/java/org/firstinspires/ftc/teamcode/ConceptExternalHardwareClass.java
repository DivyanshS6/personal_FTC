package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
@Disabled
@TeleOp(name="Concept: Robot Hardware Class", group="Robot")
public class ConceptExternalHardwareClass extends OpMode {

    RobotHardware robot = new RobotHardware();
    double handOffset = 0;

    @Override
    public void init() {
        robot.init(this); // Initialize hardware
    }

    @Override
    public void loop() {
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        robot.setDrivePower(leftPower, rightPower);

        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.update();
    }
}

/*
    -------------------------------
    LEGACY CODE MOVED TO BOTTOM
    -------------------------------

    // Example implementation if using LinearOpMode:
    //
    // public void runOpMode() {
    //     robot.init(this);
    //     waitForStart();
    //     while (opModeIsActive()) {
    //         double leftPower = -gamepad1.left_stick_y;
    //         double rightPower = -gamepad1.right_stick_y;
    //         robot.setDrivePower(leftPower, rightPower);
    //
    //         if (gamepad1.right_bumper)
    //             handOffset += robot.HAND_SPEED;
    //         else if (gamepad1.left_bumper)
    //             handOffset -= robot.HAND_SPEED;
    //         handOffset = Range.clip(handOffset, -0.5, 0.5);
    //         robot.setHandPositions(handOffset);
    //
    //         if (gamepad1.y)
    //             robot.setArmPower(robot.ARM_UP_POWER);
    //         else if (gamepad1.a)
    //             robot.setArmPower(robot.ARM_DOWN_POWER);
    //         else
    //             robot.setArmPower(0);
    //
    //         telemetry.update();
    //         sleep(50);
    //     }
    // }

    // This OpMode demonstrates use of external hardware abstraction.
    // Make sure all hardware names in config match names in RobotHardware.java:
    // - left_front_drive
    // - left_back_drive
    // - right_front_drive
    // - right_back_drive
    // Optional: arm, left_hand, right_hand
*/
