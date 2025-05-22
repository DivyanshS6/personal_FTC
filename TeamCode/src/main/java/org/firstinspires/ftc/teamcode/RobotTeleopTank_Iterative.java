package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Basic Tank TeleOp", group = "Robot") // this tells the driver station app that this is the teleop mode
public class RobotTeleopTank_Iterative extends OpMode { // init() runs once at start when you press init in driver station, loop() runs continuously and stop() runs when stop is pressed

    // Declare motors (initializes them)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    public double leftPower;
    public double rightPower;
    public double scalar;
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize motors from hardware map
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive"); // looks for a motor named left_front_drive
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // make sure that the robot is wired correctly if need change this to change direction
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD); // farward
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized"); // SO COOOL :DDDDDDDD
        telemetry.update(); // allegedly not needed am not sure just keep to make sure I guess

    }

    @Override
    public void loop() {

        leftPower = -gamepad1.left_stick_y - (-gamepad1.right_stick_x); // gamepad 1 is the primary controller
        rightPower = -gamepad1.left_stick_y - gamepad1.right_stick_x;

        //poo linearop

        // Normalize
        scalar = max((Math.abs(leftPower)),Math.abs(rightPower));

        if(scalar > 1) {
            // divide by max value to scale the max value to be exactly 1 and scale other value to keep the same ratio as there both divided by same value
            leftPower /= scalar;
            rightPower /= scalar;
        }


        // inverts axis as joysticks return -1 forward and 1 backward, this makes robot go forward when we push forwards (to us) the joystick
        // this is cool because the amount you push the gamepad will be the speed :D

        /*
        double speedModifier = 0.5; // Adjust this value as needed
        double leftPower = -gamepad1.left_stick_y * speedModifier;
        double rightPower = -gamepad1.right_stick_y * speedModifier;
        */

        leftFrontDrive.setPower(leftPower); // applies power to each motor
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);

        telemetry.addData("Left Power", "%.2f", leftPower); // remove if hard to read
        telemetry.addData("Right Power", "%.2f", rightPower); // will see in the driver station app
        telemetry.update();
    }
}
/* future reference

the following is yt video's configuration as to how to connect each motor
_______________________________________________________________________________
🔧 How to Connect Motors in Hardware
Step-by-step:
Open the FTC Robot Controller app on your robot phone.

Go to Configure Robot → New Configuration.

For each motor:

Add a DC Motor.

Name it exactly what your code expects, e.g.:

left_front_drive

left_back_drive

right_front_drive

right_back_drive

Select the port on your Expansion Hub or Control Hub that it’s connected to.

Ex: Port 0 for left_front_drive, etc.

Save your configuration (give it a name like TankDriveBot) and activate it.

⚙️ How to Connect Motors Physically
You will use REV Expansion Hub or Control Hub with Andymark or NeveRest motors.

Plug each motor into the correct motor port (0–3 typically).

Match the ports with the names in your configuration:

Ex: Port 0 = left_front_drive

Use standard 4mm bullet connectors to wire them to motor cables.

Keep your battery plugged in for power.

🧠 Wiring Tip:
If your robot drives crooked:

Check motor directions.

Adjust one or more with .setDirection(DcMotor.Direction.REVERSE);

🧪 Test the Code
Install the app & connect your Driver Station.

Select your saved configuration.

Select "Basic Tank TeleOp" on the Driver Station.

Press INIT, then START.

Push joysticks to drive each side.
 */