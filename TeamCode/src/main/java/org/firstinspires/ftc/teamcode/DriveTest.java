package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.Date;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="DriveTest", group="Tests")

public class DriveTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime timeSinceToggle = new ElapsedTime();
    private DcMotor motor = null;
    private DcMotor motor2 = null;
    private DcMotor triggerMotor = null;
    private DcMotor intake = null;

    boolean isSpinning = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        motor  = hardwareMap.get(DcMotor.class, "motor");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        triggerMotor = hardwareMap.get(DcMotor.class, "triggerMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //triggerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        triggerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        triggerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        timeSinceToggle.reset();


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override

    public void loop() {
        if (gamepad1.x) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.a) {
            triggerMotor.setPower(-0.5);
        } else if (gamepad1.b) {
            triggerMotor.setPower(0.5);
        } else {
            triggerMotor.setPower(0);
        }

        if (isSpinning) {
            motor.setPower(1);
            motor2.setPower(-1);
        } else {
            motor.setPower(0);
            motor2.setPower(0);
        }

        if (gamepad1.left_bumper && isSpinning && timeSinceToggle.milliseconds() > 300) {
            isSpinning = false;
            timeSinceToggle.reset();
        } else if (gamepad1.left_bumper && !isSpinning && timeSinceToggle.milliseconds() > 300) {
            isSpinning = true;
            timeSinceToggle.reset();
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


}