package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Disabled
public class RobotHardware {

    private OpMode myOpMode = null;  // Reference to OpMode to access telemetry, hardwareMap, etc.

    // Drive motors
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    // Constants
    public static final double ARM_UP_POWER    =  0.45;
    public static final double ARM_DOWN_POWER  = -0.45;

    // Initialize hardware
    public void init(OpMode opMode) {
        myOpMode = opMode;
        HardwareMap hardwareMap = myOpMode.hardwareMap;

        // Get motors from hardware map
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Motor directions (adjust as needed)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    // Set drive power for left and right sides
    public void setDrivePower(double leftPower, double rightPower) {
        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
    }

    /*
    // Optional future methods for arm/servo use

    private DcMotor armMotor = null;
    private Servo leftHand = null;
    private Servo rightHand = null;

    public static final double MID_SERVO = 0.5;
    public static final double HAND_SPEED = 0.02;

    public void setArmPower(double power) {
        armMotor.setPower(power);
    }

    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);
    }
    */
}
