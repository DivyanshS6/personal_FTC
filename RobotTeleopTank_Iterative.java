package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Basic Tank TeleOp V4.2", group = "Robot") // name passed through group parameter in the annotation (java feature)
public class RobotTeleopTank_Iterative extends OpMode { // java inheritance

    // Drive motors
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    // Arm components
    private DcMotor pivotMotor = null;
    private DcMotor elevatorMotor = null;
    private Servo clawMotor = null;

    public double leftPower;
    public double rightPower;
    public double scalar;

    // Enums for positions
    enum PivotPosition { // so we dont have a bunch of numbers and we know what number means what (no magic numbers)
        HOME(0),
        ALGAE_GRAB(300),
        ALGAER_GRAB(600),
        SCORE(50),
        SCORE_BACK(50); // entries

        final int pos; // final means wont change, and the previous are dedicated numbers I can utilize
        PivotPosition(int pos) { this.pos = pos; } // sets current instance i.e when home is called to 0
    }

    enum ElevatorPosition {
        HOME(0),
        ALGAE_GRAB(200),
        ALGAER_GRAB(400),
        STOW(600),
        SCORE(800),
        SCORE_BACK(650); // all of the above is pos

        final int pos; // never changes we only select which one to use
        ElevatorPosition(int pos) { this.pos = pos; } // constructor that sets the pos
    }

    enum ClawPosition { // servo motors operate from 0.0 (fully open) to 1.0 (fully closed)
        OPEN(0.0),
        ALGAER_CLOSE(0.5),
        ALGAE_CLOSE(1.0);

        final double pos;
        ClawPosition(double pos) { this.pos = pos; } // when something like open is created it stores the number inside of pos
        // this.pos break down to this which is what we called like OPEN and .pos is the parameter inside the brackets OPEN(0.0) in that 0.0 is the parameter
    } // this is omega cool advanced enum syntax called extended Enum syntax

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        pivotMotor = hardwareMap.get(DcMotor.class, "pivot_motor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elvevator_motor");
        clawMotor = hardwareMap.get(Servo.class, "claw_motor");

        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        // Start at home positions
        pivotMotor.setTargetPosition(PivotPosition.HOME.pos);
        pivotMotor.setPower(1.0);

        elevatorMotor.setTargetPosition(ElevatorPosition.HOME.pos);
        elevatorMotor.setPower(1.0);

        clawMotor.setPosition(ClawPosition.OPEN.pos);

        // Reset encoder and set mode
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Driving
        leftPower = -gamepad1.left_stick_y + gamepad1.left_stick_x;
        rightPower = -gamepad1.left_stick_y - gamepad1.left_stick_x;

        scalar = max(Math.abs(leftPower), Math.abs(rightPower));
        if (scalar > 1) {
            leftPower /= scalar;
            rightPower /= scalar;
        }

        leftBackDrive.setPower(leftPower);
        rightBackDrive.setPower(rightPower);

        // --- Pivot Motor Controls ---
        if (gamepad1.dpad_up) {
            pivotMotor.setTargetPosition(PivotPosition.SCORE.pos);

        } else if (gamepad1.dpad_down) {
            pivotMotor.setTargetPosition(PivotPosition.HOME.pos);
        } else if (gamepad1.dpad_left) {
            pivotMotor.setTargetPosition(PivotPosition.ALGAE_GRAB.pos);
        } else if (gamepad1.dpad_right) {
            pivotMotor.setTargetPosition(PivotPosition.ALGAER_GRAB.pos);
        } else if (gamepad1.left_bumper) {
            pivotMotor.setTargetPosition(PivotPosition.SCORE_BACK.pos);
        }
        pivotMotor.setPower(1.0); // constant power to hold position

        // --- Elevator Motor Controls ---
        if (gamepad1.y) {
            elevatorMotor.setTargetPosition(ElevatorPosition.SCORE.pos);
            telemetry.addData("Elevator Position", ElevatorPosition.SCORE.name());
        } else if (gamepad1.a) {
            elevatorMotor.setTargetPosition(ElevatorPosition.HOME.pos);
            telemetry.addData("Elevator Position", ElevatorPosition.HOME.name());
        } else if (gamepad1.x) {
            elevatorMotor.setTargetPosition(ElevatorPosition.ALGAE_GRAB.pos);
            telemetry.addData("Elevator Position", ElevatorPosition.ALGAER_GRAB.name());
        } else if (gamepad1.b) {
            elevatorMotor.setTargetPosition(ElevatorPosition.ALGAER_GRAB.pos);
            telemetry.addData("Elevator Position", ElevatorPosition.ALGAER_GRAB.name());
        } else if (gamepad1.right_bumper) {
            elevatorMotor.setTargetPosition(ElevatorPosition.SCORE_BACK.pos);
            telemetry.addData("Elevator Position", ElevatorPosition.SCORE_BACK.name());
        } else if (gamepad1.start) {
            elevatorMotor.setTargetPosition(ElevatorPosition.STOW.pos);
            telemetry.addData("Elevator Position", ElevatorPosition.STOW.name());
        }
        elevatorMotor.setPower(1.0);

        // --- Claw Servo Controls ---
        if (gamepad1.left_trigger > 0.2) {
            clawMotor.setPosition(ClawPosition.ALGAE_CLOSE.pos);
            telemetry.addData("Claw Position", ClawPosition.ALGAE_CLOSE.name());
        } else if (gamepad1.right_trigger > 0.2) {
            clawMotor.setPosition(ClawPosition.ALGAER_CLOSE.pos);
            telemetry.addData("Claw Position", ClawPosition.ALGAER_CLOSE.name());
        } else if (gamepad1.back) {
            clawMotor.setPosition(ClawPosition.OPEN.pos);
            telemetry.addData("Claw Position", ClawPosition.OPEN.name()); // add for each im too lazy
        }


        telemetry.addLine();
        telemetry.addLine(); // i think this adds a new line, i could be wrong...

        telemetry.addData("Pivot Target", pivotMotor.getTargetPosition());
        telemetry.addData("Elevator Target", elevatorMotor.getTargetPosition());
        telemetry.addData("Claw Position", clawMotor.getPosition());

        telemetry.update();
    }
}
