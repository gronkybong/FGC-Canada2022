package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@TeleOp(name= "ShooterTest 2", group="Tests")

public class ShooterTest2 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor = null;
    private DcMotor motor2 = null;
    private DcMotor triggerMotor = null;
    //private Servo trigger = null;
    double speed;
    int currentPos;
    int lastPos;

    double lastTime;
    double startShoot;
    double low;
    int avgRunningSum = 0;
    int iterations;
    boolean shot = false;
    int targetPosition = -40;

//    OutputStreamWriter shooterWriter;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motor  = hardwareMap.get(DcMotor.class, "motor");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        triggerMotor = hardwareMap.get(DcMotor.class, "triggerMotor");

        triggerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
//        try {
//            shooterWriter = new FileWriter("/sdcard/FIRST/shooterlog_" + java.text.DateFormat.getDateTimeInstance().format(new Date()) + ".csv", true);
//        } catch (IOException e) {
//            throw new RuntimeException("shooter log file writer open failed: " + e.toString());
//        }
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
        runtime.reset();


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    int rpm = 0;
    int rpmSum;
    int lastAvgRpm;

    @Override

    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double power = gamepad1.right_trigger - gamepad1.left_trigger;

        motor.setPower(power);
        motor2.setPower(-power *1);
        currentPos = motor.getCurrentPosition();

        speed = Math.abs(currentPos - lastPos) / (runtime.milliseconds() - lastTime);
        lastTime = runtime.milliseconds();
        rpm = (int)(speed*1000*60/28);

        rpmSum+= rpm;

        if(iterations % 20 == 0)
        {
            lastAvgRpm = rpmSum / 20;
            rpmSum = 0;
//            try {
//                RobotLog.d("shooterWriter.write");
//                shooterWriter.write(String.format("%.0f, %d\n", runtime.milliseconds(), lastAvgRpm));
//            } catch (IOException e) {
//                throw new RuntimeException("shooter log file writer write failed: " + e.toString());
//            }
        }

//        if(gamepad1.right_bumper)
//        {
//            shot = true;
//            low = lastAvgRpm;
//            avgRunningSum = lastAvgRpm;
//            trigger.setPosition(0);
//            startShoot = runtime.milliseconds();
//        }
//        if(gamepad1.left_bumper)
//            trigger.setPosition(0.3);

        if(gamepad1.right_bumper)
        {
            shot = true;
            low = lastAvgRpm;
            avgRunningSum = lastAvgRpm;
            startShoot = runtime.milliseconds();

            triggerMotor.setTargetPosition(targetPosition);
            triggerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            triggerMotor.setPower(-1);
        }

        if(gamepad1.left_bumper)
        {
            triggerMotor.setTargetPosition(0);
            triggerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            triggerMotor.setPower(1);
        }

        lastPos = currentPos;
        iterations++;

        double time = 0;

        if(gamepad1.a)
        {
            avgRunningSum = 0;
        }
        if(shot)
        {
            if(lastAvgRpm < low)
                low = lastAvgRpm;

            if(lastAvgRpm >= avgRunningSum){
                time = runtime.milliseconds() - startShoot;
                avgRunningSum = 0;
                shot = false;
            }

        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motor RPM", lastAvgRpm);
        telemetry.addData("Motor Speed", "%.2f", (float)speed);
        telemetry.addData("Pos","%.2f", (float)currentPos);
        telemetry.addData("low","%.2f", (float)low);
        telemetry.addData("recovery time","%.2f", (float)time);
        telemetry.addData("total avg", avgRunningSum);
        telemetry.addData("trigger pos", triggerMotor.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //close();
    }

//    public void close(){
//        try {
//            RobotLog.d("shooter log Writer.close");
//            shooterWriter.close();
//        } catch (IOException e) {
//            throw new RuntimeException("shooter log file writer close failed: " + e.toString());
//        }
//    }

}