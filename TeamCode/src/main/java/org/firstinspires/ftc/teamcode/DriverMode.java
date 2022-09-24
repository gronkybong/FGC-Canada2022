package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.RevRoboticsCoreHexMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name ="DriverMode", group="Tests")

public class DriverMode extends OpMode {
    private DcMotorEx shooterMotor1 = null;
    private DcMotorEx shooterMotor2 = null;
    private DcMotor triggerMotor = null;
    private DcMotor intake = null;
    private DcMotor driveMotor1 = null;
    private DcMotor driveMotor2 = null;
    private DcMotor winch = null;
    private DcMotor linearSlide = null;
    private ElapsedTime timeSinceToggle = new ElapsedTime();

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    boolean shooterOn = true;
    private double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        //configuration
        shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
        triggerMotor = hardwareMap.get(DcMotor.class, "triggerMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        driveMotor1 = hardwareMap.get(DcMotorEx.class, "driveMotor1");
        driveMotor2 = hardwareMap.get(DcMotorEx.class, "driveMotor2");
        winch = hardwareMap.get(DcMotor.class, "winch");
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");

        //direction
        shooterMotor1.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        triggerMotor.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        driveMotor1.setDirection(DcMotor.Direction.REVERSE);
        driveMotor2.setDirection(DcMotor.Direction.FORWARD);
        winch.setDirection(DcMotor.Direction.FORWARD);
        linearSlide.setDirection(DcMotor.Direction.FORWARD);

        //Encoder Usage
        triggerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double power = PIDControl (1000, shooterMotor1.getVelocity());
        shooterMotor1.setPower(power);
    }

    public double PIDControl (double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }

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

    @Override
    public void loop() {

        //Drive Train
        driveMotor1.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        driveMotor2.setPower(gamepad1.left_stick_x + gamepad1.left_stick_x);

        telemetry..addData("Motor Speed", "%.2f", (float)speed);

        //Winch Controls
        if (gamepad1.a) {
            winch.setPower(1);
        } else if (gamepad1.b) {
            winch.setPower(-1);
        } else {
            winch.setPower(0);
        }

        //LinearSlide Controls
        if (gamepad1.x) {
            linearSlide.setPower(1);
        } else if (gamepad1.y) {
            linearSlide.setPower(-1);
        } else {
            linearSlide.setPower(0);
        }

        //intake Controls
        if (gamepad1.left_bumper) {
            intake.setPower(1);
        } else if (gamepad1.right_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        //Trigger Controls
        if (gamepad1.left_trigger) {
            triggerMotor.setPower(0.5);
        } else if (gamepad1.right_trigger) {
            triggerMotor.setPower(-0.5);
        } else {
            triggerMotor.setPower(0);
        }

        //Shooter Controls
        if (gamepad1.dpad_up && shooterOn && timeSinceToggle.milliseconds() > 300) {
            shooterOn = true;
            timeSinceToggle.reset();
        } else if (gamepad1.dpad_up && !shooterOn && timeSinceToggle.milliseconds() > 300) {
            shooterOn = false;
            timeSinceToggle.reset();
        }
    }


    @Override
    public void stop() {
    }

}
