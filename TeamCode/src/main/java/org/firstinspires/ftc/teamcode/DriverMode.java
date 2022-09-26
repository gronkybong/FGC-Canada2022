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
    private Servo lowBarLeft = null;
    private Servo lowBarRight = null;
    private Servo winchLock = null;
    private ElapsedTime timeSinceToggle = new ElapsedTime();
    private ElapsedTime timeSinceToggle2 = new ElapsedTime();
    private ElapsedTime timeSinceToggle3 = new ElapsedTime();

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    boolean shooterOn = false;
    boolean slideWinchSync = true;
    boolean isWinchLocked = false;
    private double lastError = 0;

    private final int winchMaxTicks = 2200000;
    private int currentWinchTicks;
    private final int slideMaxTicks = 540000;
    private int currentSlideTicks;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        //configuration
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "BShooterMotor");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "FShooterMotor");
        triggerMotor = hardwareMap.get(DcMotor.class, "TriggerMotor");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        driveMotor1 = hardwareMap.get(DcMotorEx.class, "RightDriveMotor");
        driveMotor2 = hardwareMap.get(DcMotorEx.class, "LeftDriveMotor");
        winch = hardwareMap.get(DcMotor.class, "Winch");
        linearSlide = hardwareMap.get(DcMotor.class, "LinearSlide");
        lowBarLeft = hardwareMap.get(Servo.class, "LowBar1");
        lowBarLeft.setPosition(0.5);
        lowBarRight = hardwareMap.get(Servo.class, "LowBar2");
        lowBarRight.setPosition(0.5);
        winchLock = hardwareMap.get(Servo.class, "WinchLock");
        winchLock.setPosition(0.5);

        //direction
        shooterMotor1.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        triggerMotor.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        driveMotor1.setDirection(DcMotor.Direction.REVERSE);
        driveMotor2.setDirection(DcMotor.Direction.FORWARD);
        winch.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setDirection(DcMotor.Direction.FORWARD);

        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Encoder Usage
        triggerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        double power = PIDControl (1000, shooterMotor1.getVelocity());
//        shooterMotor1.setPower(power);
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
        timeSinceToggle2.reset();
        timeSinceToggle3.reset();
    }

    @Override
    public void loop() {

        //Drive Train
        driveMotor1.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x);
        driveMotor2.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x);
//        driveMotor1.setPower(gamepad1.left_stick_y);
//        driveMotor2.setPower(gamepad1.right_stick_y);
//        telemetry.addData("left", gamepad1.left_stick_y);
//        telemetry.addData("right", gamepad1.right_stick_y);

        //telemetry.addData("Motor Speed", "%.2f", (float)speed);

        //intake Controls
        if (gamepad1.left_bumper) {
            intake.setPower(1);
        } else if (gamepad1.right_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        //Trigger Controls
        if (gamepad1.left_trigger > 0) {
            triggerMotor.setPower(0.5);
        } else if (gamepad1.right_trigger > 0) {
            triggerMotor.setPower(-0.5);
        } else {
            triggerMotor.setPower(0);
        }

        //Shooter Controls
        if (gamepad1.touchpad && shooterOn && timeSinceToggle.milliseconds() > 300) {
            shooterOn = false;
            timeSinceToggle.reset();
        } else if (gamepad1.touchpad && !shooterOn && timeSinceToggle.milliseconds() > 300) {
            shooterOn = true;
            timeSinceToggle.reset();
        }
        if (shooterOn) {
            shooterMotor1.setPower(-1);
            shooterMotor2.setPower(1);
        } else {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
        }

        //low bar controls
        if (gamepad1.dpad_up) {
            lowBarLeft.setPosition(lowBarLeft.getPosition() + 0.005);
            lowBarRight.setPosition(lowBarRight.getPosition() - 0.005);
        } else if (gamepad1.dpad_down) {
            lowBarLeft.setPosition(lowBarLeft.getPosition() - 0.005);
            lowBarRight.setPosition(lowBarRight.getPosition() + 0.005);
        }

        //lock controls
        if (gamepad1.dpad_left && isWinchLocked && timeSinceToggle3.milliseconds() > 300) {
            isWinchLocked = false;
            timeSinceToggle3.reset();
        } else if (gamepad1.dpad_left && !isWinchLocked && timeSinceToggle3.milliseconds() > 300) {
            isWinchLocked = true;
            timeSinceToggle3.reset();
        }
        if (isWinchLocked) {
            winchLock.setPosition(0.55);
        } else {
            winchLock.setPosition(0.5);
        }

        //Winch Controls
        currentWinchTicks = winch.getCurrentPosition();
        currentSlideTicks = linearSlide.getCurrentPosition();
        if (!slideWinchSync) {
            if (gamepad1.cross && currentWinchTicks > -winchMaxTicks) {
                winch.setPower(1);
            } else if (gamepad1.circle && !isWinchLocked) {
                winch.setPower(-1);
            } else {
                winch.setPower(0);
            }

            //LinearSlide Controls
            if (gamepad1.triangle && currentSlideTicks < slideMaxTicks) {
                linearSlide.setPower(1);
            } else if (gamepad1.square && currentSlideTicks > 0) {
                linearSlide.setPower(-1);
            } else {
                linearSlide.setPower(0);
            }
        }

        //winch & slide COMBINED controls
        if (gamepad1.right_stick_button && timeSinceToggle2.milliseconds() > 300 && slideWinchSync) {
            slideWinchSync = false;
            timeSinceToggle2.reset();
        } else if (gamepad1.right_stick_button && timeSinceToggle2.milliseconds() > 300 && !slideWinchSync) {
            slideWinchSync = true;
            timeSinceToggle2.reset();
        }

        if (gamepad1.triangle && currentSlideTicks < slideMaxTicks) {
            linearSlide.setPower(0.95);
            winch.setPower(1);
        } else if (gamepad1.square && currentSlideTicks > 0) {
            linearSlide.setPower(-0.95);
            winch.setPower(-1);
        } else {
            linearSlide.setPower(0);
        }

        telemetry.addData("Winch Ticks", winch.getCurrentPosition());
        telemetry.addData("Slide Ticks", linearSlide.getCurrentPosition());
    }


    @Override
    public void stop() {
    }

}
