package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Basic Tank TeleOp V.twelve.engine DEF", group = "Robot") // name passed through group parameter in the annotation (java feature)
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

    public int elevatorNum;
    // Enums for positions
    enum PivotPosition { // so we dont have a bunch of numbers and we know what number means what (no magic numbers)
        HOME(350),
        ALGAE_GRAB(25),
        ALGAER_GRAB(35),
        SCORE(100),
        CLIMB(-20);

        final int pos; // final means wont change, and the previous are dedicated numbers I can utilize
        PivotPosition(int pos) {
            this.pos = pos;
        } // sets current instance i.e when home is called to 0
    }

    enum ElevatorPosition {
        HOME(0),
        ALGAE_GRAB(0),
        ALGAER_GRAB(0),
        CLIMB(0),
        SCORE(1000); // all of the above is pos

        final int pos; // never changes we only select which one to use
        ElevatorPosition(int pos) { this.pos = pos; } // constructor that sets the pos
    }

    enum ClawPosition { // servo motors operate from 0.0 (fully open) to 1.0 (fully closed)
        OPEN(0.75),
        ALGAER_CLOSE(1.0),
        ALGAE_CLOSE(1.0),
        CLIMB(1.0);

        final double pos;
        ClawPosition(double pos) { this.pos = pos; } // when something like open is created it stores the number inside of pos
        // this.pos break down to this which is what we called like OPEN and .pos is the parameter inside the brackets OPEN(0.0) in that 0.0 is the parameter
    } // this is omega cool advanced enum syntax called extended Enum syntax

    // Helper Meth
    private double getScaledPower(int error) {
        double absError = Math.abs(error); // must use error as that is the official defining word

        double kP = 0.0015; // Tune dis value for da scalene triangle
        double power = kP * absError;

        // for clamp
        power = Math.min(power, 1.0); // compares the 2 values and returns the smaller one
        power = Math.max(power, 0.1); // same as above but returns the bigger one


        return (error > 0) ? power : -power; // kepp direction (flip if wrong idk)
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        pivotMotor = hardwareMap.get(DcMotor.class, "pivot_motor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevator_motor"); // Fixed typo: was "elvevator_motor"
        clawMotor = hardwareMap.get(Servo.class, "claw_motor");

        // Start at home positions
        pivotMotor.setTargetPosition(PivotPosition.HOME.pos);
        pivotMotor.setPower(0.5); // change in future to 0.75


        clawMotor.setPosition(ClawPosition.OPEN.pos);




        // Reset encoder and set mode
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setTargetPosition(0);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotor.setPower(0);

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        elevatorNum = elevatorMotor.getCurrentPosition();
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


        if (gamepad1.x){
            pivotMotor.setTargetPosition(PivotPosition.HOME.pos);
            elevatorMotor.setTargetPosition(ElevatorPosition.HOME.pos);
            clawMotor.setPosition(ClawPosition.ALGAER_CLOSE.pos);

            pivotMotor.setPower(0.5);
            elevatorMotor.setPower(0);

            telemetry.addData("Pivot Position", PivotPosition.HOME.name());
            telemetry.addData("Elevator Position", ElevatorPosition.HOME.pos);
            telemetry.addData("Claw Position", ClawPosition.ALGAER_CLOSE.name());

        } else if (gamepad1.right_bumper){
            pivotMotor.setTargetPosition(PivotPosition.ALGAE_GRAB.pos);
            elevatorMotor.setTargetPosition(ElevatorPosition.ALGAE_GRAB.pos);
            clawMotor.setPosition(ClawPosition.OPEN.pos);

            pivotMotor.setPower(0.5);
            elevatorMotor.setPower(0);

            telemetry.addData("Pivot Position", PivotPosition.ALGAE_GRAB.name());
            telemetry.addData("Elevator Position", ElevatorPosition.ALGAE_GRAB.pos);
            telemetry.addData("Claw Position", ClawPosition.OPEN.name());

        } else if (gamepad1.y){
            pivotMotor.setTargetPosition(PivotPosition.SCORE.pos);
            elevatorMotor.setTargetPosition(ElevatorPosition.SCORE.pos);
            clawMotor.setPosition(ClawPosition.ALGAER_CLOSE.pos);

            pivotMotor.setPower(0.5);
            elevatorMotor.setPower(0);

            telemetry.addData("Pivot Position", PivotPosition.SCORE.name());
            telemetry.addData("Elevator Position", ElevatorPosition.SCORE.pos);
            telemetry.addData("Claw Position", ClawPosition.ALGAER_CLOSE.name());

        } else if (gamepad1.b) {
            pivotMotor.setTargetPosition(PivotPosition.CLIMB.pos);
            clawMotor.setPosition(ClawPosition.CLIMB.pos);
            elevatorMotor.setTargetPosition(ElevatorPosition.CLIMB.pos);

            telemetry.addData("Pivot Position", PivotPosition.CLIMB.name());
            telemetry.addData("Elevator Position", ElevatorPosition.CLIMB.pos);
            telemetry.addData("Claw Position", ClawPosition.CLIMB.name());
        } else if (gamepad1.left_bumper) {

            pivotMotor.setTargetPosition(PivotPosition.ALGAER_GRAB.pos);
            elevatorMotor.setTargetPosition(ElevatorPosition.ALGAER_GRAB.pos);
            clawMotor.setPosition(ClawPosition.OPEN.pos);

            pivotMotor.setPower(0.5);
            elevatorMotor.setPower(0);

            telemetry.addData("Pivot Position", PivotPosition.ALGAER_GRAB.name());
            telemetry.addData("Elevator Position", ElevatorPosition.ALGAER_GRAB.pos);
            telemetry.addData("Claw Position", ClawPosition.OPEN.name());
        }
        else if (gamepad1.left_trigger > 0.2) {
            clawMotor.setPosition(ClawPosition.OPEN.pos);

            telemetry.addData("Claw Position", ClawPosition.OPEN.name());

        } else if (gamepad1.right_trigger > 0.2) {
            clawMotor.setPosition(ClawPosition.ALGAE_CLOSE.pos);

            telemetry.addData("Claw Position", ClawPosition.ALGAE_CLOSE);
        }

        // elevator adjustment
        if (gamepad1.dpad_up) {
            elevatorNum += 10;
            elevatorMotor.setTargetPosition(elevatorNum);
            elevatorMotor.setPower(0.5);
        } else if (gamepad1.dpad_down) {
            elevatorNum -= 10;
            elevatorMotor.setTargetPosition(elevatorNum);
            elevatorMotor.setPower(0.5);
        } else {
            if (!elevatorMotor.isBusy()) {
                elevatorMotor.setPower(0);
            }
        }


// pid control for pivot motor
        if (pivotMotor.isBusy()) {
            int error = pivotMotor.getTargetPosition() - pivotMotor.getCurrentPosition(); // makes sure error is recalculated everytime buttt maybe remove the int again
            pivotMotor.setPower(getScaledPower(error));
        } else {
            pivotMotor.setPower(0);
        }


/*
        if ((pivotMotor.getCurrentPosition() <= pivotMotor.getTargetPosition() && pivotMotor.getCurrentPosition() >= pivotMotor.getTargetPosition() - 20)
                ||
                (pivotMotor.getCurrentPosition() >= pivotMotor.getTargetPosition() && pivotMotor.getCurrentPosition() <= pivotMotor.getTargetPosition() + 20)) {
            int error = elevatorMotor.getTargetPosition() - elevatorMotor.getCurrentPosition();
            elevatorMotor.setPower(getScaledPower(error));
        } else {
            elevatorMotor.setPower(0);
        }
*/

        // that was more complex than it needs to be rather I should do:




        // to make elevator motor go only once the bigger arm rotation is completed

        int pivotError = Math.abs(pivotMotor.getTargetPosition() - pivotMotor.getCurrentPosition());

        if (pivotError <= 20) {
            int error = elevatorMotor.getTargetPosition() - elevatorMotor.getCurrentPosition();
            elevatorMotor.setPower(getScaledPower(error));
        } else {
            elevatorMotor.setPower(0);
        }



        telemetry.addData("Claw Position (Raw)", clawMotor.getPosition());
        telemetry.addData("Elevator Busy", elevatorMotor.isBusy());
        telemetry.addData("Pivot Busy", pivotMotor.isBusy());
        telemetry.addData("Elevator RAW num: ", elevatorMotor.getCurrentPosition());
        telemetry.addData("ElevatorNum pos: ", elevatorNum);

        telemetry.update();
    }
}
