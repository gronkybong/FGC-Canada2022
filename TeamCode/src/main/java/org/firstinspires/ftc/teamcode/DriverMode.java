package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

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
    private CRServo lowBarLeft = null;
    private CRServo lowBarRight = null;
    private Servo winchLock = null;
    private ElapsedTime timeSinceToggle = new ElapsedTime();
    private ElapsedTime timeSinceToggle2 = new ElapsedTime();
    private ElapsedTime timeSinceToggle3 = new ElapsedTime();
    private ElapsedTime shooterTimer = new ElapsedTime();

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    boolean shooterOn = false;
    boolean slideWinchSync = true;
    boolean isWinchLocked = false;
    boolean slideAuto = false;
    private double lastError = 0;

    private int winchMaxTicks = 12350;
    private int currentWinchTicks;
    private final int slideMaxTicksMax = 2000;
    private final int slideMaxTicksMid = 1300;
    private int currentSlideTicks;

    private double slideWinchConversion = 6.5;
    double drivePower = 0.75;
    double leftStickYTranslated;

    int lastShooterTick1 = 0;
    int lastShooterTick2 = 0;
    int currentShooterTick1;
    int currentShooterTick2;
    int currentShooterRPM1;
    int currentShooterRPM2;

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
        lowBarLeft = hardwareMap.get(CRServo.class, "LowBar1");
        //lowBarLeft.setPosition(1);
        lowBarLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        lowBarRight = hardwareMap.get(CRServo.class, "LowBar2");
        //lowBarRight.setPosition(0);
        lowBarRight.setDirection(DcMotorSimple.Direction.REVERSE);
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
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoder Usage
        triggerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        if (gamepad1.left_trigger > 0) {
            drivePower = 0.5;
        } else if (gamepad1.right_trigger > 0) {
            drivePower = 1;
        } else {
            drivePower = 0.65;
        }

        driveMotor1.setPower((-gamepad1.left_stick_y - gamepad1.right_stick_x) * drivePower);
        driveMotor2.setPower((-gamepad1.left_stick_y + gamepad1.right_stick_x) * drivePower);
//        driveMotor1.setPower(gamepad1.left_stick_y);
//        driveMotor2.setPower(gamepad1.right_stick_y);
//        telemetry.addData("left", gamepad1.left_stick_y);
//        telemetry.addData("right", gamepad1.right_stick_y);

        //telemetry.addData("Motor Speed", "%.2f", (float)speed);

        //intake Controls
        if (gamepad2.left_bumper) {
            intake.setPower(1);
        } else if (gamepad2.right_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        //Trigger Controls
        if (gamepad2.left_trigger > 0) {
            triggerMotor.setPower(0.5);
        } else if (gamepad2.right_trigger > 0) {
            triggerMotor.setPower(-0.5);
        } else {
            triggerMotor.setPower(0);
        }

        //Shooter Controls
        if (gamepad2.touchpad && shooterOn && timeSinceToggle.milliseconds() > 300) {
            shooterOn = false;
            timeSinceToggle.reset();
        } else if (gamepad2.touchpad && !shooterOn && timeSinceToggle.milliseconds() > 300) {
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

        currentShooterTick1 = shooterMotor1.getCurrentPosition();
        currentShooterRPM1 = (int) (Math.abs(currentShooterTick1 - lastShooterTick1)/shooterTimer.milliseconds())*1000*60/28;
        lastShooterTick1 = currentShooterTick1;
        currentShooterTick2 = shooterMotor2.getCurrentPosition();
        currentShooterRPM2 = (int) (Math.abs(currentShooterTick2 - lastShooterTick2)/shooterTimer.milliseconds())*1000*60/28;
        lastShooterTick2 = currentShooterTick2;
        shooterTimer.reset();

        //low bar controls
//        if (gamepad2.dpad_up) {
////            lowBarLeft.setPosition(lowBarLeft.getPosition() + 0.005);
////            lowBarRight.setPosition(lowBarRight.getPosition() - 0.005);
//            lowBarLeft.setPower(0.2);
//            lowBarRight.setPower(0.2);
//        } else if (gamepad2.dpad_down) {
////            lowBarLeft.setPosition(lowBarLeft.getPosition() - 0.005);
////            lowBarRight.setPosition(lowBarRight.getPosition() + 0.005);
//            lowBarLeft.setPower(-0.2);
//            lowBarRight.setPower(-0.2);
//        } else {
//            lowBarLeft.setPower(0);
//            lowBarRight.setPower(0);
//        }
//        if (gamepad2.left_stick_y < 0.4) {
//            leftStickYTranslated = (gamepad2.left_stick_y * 2.5) - 1;
//        } else if (gamepad2.left_stick_y >= 0.4 && gamepad2.left_stick_y <= 0.6) {
//            leftStickYTranslated = 0;
//        } else if (gamepad2.left_stick_y > 0.6) {
//            leftStickYTranslated = (gamepad2.left_stick_y * 2.5) - 1.5;
//        }
//
//        lowBarLeft.setPower(leftStickYTranslated);
//        lowBarRight.setPower(leftStickYTranslated * 0.9);
        lowBarLeft.setPower(gamepad2.left_stick_y);
        lowBarRight.setPower(gamepad2.left_stick_y);

        //lock controls
//        if (gamepad1.dpad_left && isWinchLocked && timeSinceToggle3.milliseconds() > 300) {
//            isWinchLocked = false;
//            timeSinceToggle3.reset();
//        } else if (gamepad1.dpad_left && !isWinchLocked && timeSinceToggle3.milliseconds() > 300) {
//            isWinchLocked = true;
//            timeSinceToggle3.reset();
//        }
        currentWinchTicks = winch.getCurrentPosition();
        currentSlideTicks = linearSlide.getCurrentPosition();

        if (isWinchLocked) {
            winchLock.setPosition(0.54);
        } else {
            winchLock.setPosition(0.5);
        }

        if (gamepad2.triangle && currentSlideTicks < slideMaxTicksMax) {
            slideWinchSync = true;
            isWinchLocked = false;
            linearSlide.setPower(1);
        } else if (gamepad2.square && currentSlideTicks < slideMaxTicksMid) {
            slideWinchSync = true;
            isWinchLocked = false;
            linearSlide.setPower(1);
        } else if (gamepad2.circle) {
            slideWinchSync = true;
            linearSlide.setPower(-0.3);
        } else {
            slideWinchSync = false;
        }

        if (slideWinchSync) {
            if (currentSlideTicks < 500) {
                slideWinchConversion = 5.8;
            } else if (currentSlideTicks < 1000 && currentSlideTicks > 500) {
                slideWinchConversion = 6.1;
            } else if (currentSlideTicks > 1000 && currentSlideTicks < 1800) {
                slideWinchConversion = 7.1;
            } else if (currentSlideTicks > 1800) {
                slideWinchConversion = 7;
            }
            winch.setTargetPosition(-(int) slideWinchConversion * currentSlideTicks);
            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            winch.setPower(1);
        }

        if (gamepad2.dpad_down) {
            linearSlide.setPower(-0.6);
        }

        if (gamepad2.cross && timeSinceToggle2.milliseconds() > 300 && !slideAuto) {
            slideAuto = true;
            timeSinceToggle2.reset();
        } else if (gamepad2.cross && timeSinceToggle2.milliseconds() > 300 && slideAuto && !linearSlide.isBusy()) {
            slideAuto = false;
            timeSinceToggle2.reset();
        }

        if (slideAuto) {
            linearSlide.setTargetPosition(800);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(0.6);
        }

        if (gamepad1.square && currentWinchTicks > -winchMaxTicks) {
            isWinchLocked = false;
            winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            winch.setPower(-0.3);
            telemetry.addData("unwind", false);
        } else if (gamepad1.circle && currentWinchTicks < 2000) {
            isWinchLocked = true;
            winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            winch.setPower(1);
            telemetry.addData("wind", true);
        } else if (!slideWinchSync) {
            winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            winch.setPower(0);
            telemetry.addData("not doing anything", false);
        }

        if (gamepad1.triangle) {
            winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (gamepad1.dpad_down) {
                winch.setPower(0.5);
            } else if (gamepad1.dpad_up) {
                winch.setPower(-0.5);
            } else {
                winch.setPower(0);
            }
        }

        if (!(gamepad2.triangle || gamepad2.square || gamepad2.cross || gamepad2.circle || gamepad2.dpad_down || slideAuto)) {
            linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlide.setPower(0);
        }

        //Winch Controls

//        if (!slideWinchSync) {
//            winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            if (gamepad1.cross && currentWinchTicks > -winchMaxTicks && !isWinchLocked) {
//                winch.setPower(-0.3);
//                telemetry.addData("unwind", false);
//            } else if (gamepad1.circle && currentWinchTicks < 800) {
//                winch.setPower(1);
//                telemetry.addData("wind", true);
//            } else {
//                winch.setPower(0);
//                telemetry.addData("not doing anything", false);
//            }
//
//            //LinearSlide Controls
//            if (gamepad1.triangle && currentSlideTicks < slideMaxTicksMax) {
//                linearSlide.setPower(1);
//            } else if (gamepad1.square && currentSlideTicks > 0) {
//                linearSlide.setPower(-0.6);
//            } else {
//                linearSlide.setPower(0);
//            }
//        }
//
//        //winch & slide COMBINED controls
//        if (gamepad1.right_stick_button && timeSinceToggle2.milliseconds() > 300 && slideWinchSync) {
//            slideWinchSync = false;
//            timeSinceToggle2.reset();
//        } else if (gamepad1.right_stick_button && timeSinceToggle2.milliseconds() > 300 && !slideWinchSync) {
//            slideWinchSync = true;
//            timeSinceToggle2.reset();
//        }
//
//        if (slideWinchSync) {
//            if (gamepad1.triangle && currentSlideTicks < slideMaxTicksMax) {
//                linearSlide.setPower(1);
//            } else if (gamepad1.square && currentSlideTicks > 0) {
//                linearSlide.setPower(-0.3);
//            } else {
//                linearSlide.setPower(0);
//            }
//            if (currentSlideTicks < 500) {
//                slideWinchConversion = 5.7;
//            }
//            else if (currentSlideTicks < 1000 && currentSlideTicks > 500) {
//                slideWinchConversion = 6.3;
//            } else if (currentSlideTicks > 1000 && currentSlideTicks < 1800) {
//                slideWinchConversion = 7.3;
//            } else if (currentSlideTicks > 1800) {
//                slideWinchConversion = 7.1;
//            }
//            winch.setTargetPosition(-(int) slideWinchConversion * currentSlideTicks);
//            winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            winch.setPower(1);
//            //-12200
//        }

//        if (slideWinchSync) {
//            if (gamepad1.triangle && currentSlideTicks < slideMaxTicks) {
//                linearSlide.setPower(0.6);
//                winch.setPower(-0.2);
//            } else if (gamepad1.square && currentSlideTicks > 0) {
//                linearSlide.setPower(-0.95);
//                winch.setPower(1);
//            } else {
//                linearSlide.setPower(0);
//                winch.setPower(0);
//            }
//        }
//        telemetry.addData("Winch Ticks", winch.getCurrentPosition());
//        telemetry.addData("Slide Ticks", linearSlide.getCurrentPosition());
//        telemetry.addData("winch lock:", isWinchLocked);
//        telemetry.addData("sync:", slideWinchSync);
//        telemetry.addData("ratio:", slideWinchConversion);
//        telemetry.addData("left stick 2", gamepad2.left_stick_y);
//        telemetry.addData("left stick translated", leftStickYTranslated);
        telemetry.addData("shooter1", currentShooterRPM1);
        telemetry.addData("shooter2", currentShooterRPM2);
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}
