package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Slide Test", group="Tests")

public class SlideTest extends OpMode{

    /* Declare OpMode members. */
    private DcMotor motor = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.left_trigger - gamepad1.right_trigger != 0) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower((gamepad1.left_trigger - gamepad1.right_trigger)*1);
        }

        if (gamepad1.touchpad_finger_1) {
            motor.setTargetPosition((int) ((gamepad1.touchpad_finger_1_x + 1) * 1000));
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad1.dpad_left) {
            motor.setTargetPosition(2000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad1.dpad_up) {
            motor.setTargetPosition(1000);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("position",  "%d", motor.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}