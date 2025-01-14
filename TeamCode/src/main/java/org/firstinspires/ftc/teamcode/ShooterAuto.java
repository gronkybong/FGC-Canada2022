/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@Autonomous(name="Shooter Auto", group="Tests")
public class ShooterAuto extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor motor = null;
    private DcMotor motor2 = null;
    private DcMotor triggerMotor = null;

    int targetPosition = -35;
    int holdWait = 500;
    int intervalWait = 1000;
    double power = 1;

    @Override
    public void runOpMode() {

        motor  = hardwareMap.get(DcMotor.class, "motor");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        triggerMotor = hardwareMap.get(DcMotor.class, "triggerMotor");

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        triggerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        triggerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        triggerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        sleep(2000);
        motor.setPower(power);
        motor2.setPower(-power);
        sleep(30000);
//        while (triggerMotor.getCurrentPosition() > targetPosition) {
//            triggerMotor.setPower(-1);
//        }
//        triggerMotor.setPower(0);
//        sleep(holdWait);
//        while (triggerMotor.getCurrentPosition() < 0) {
//            triggerMotor.setPower(1);
//        }
//        triggerMotor.setPower(0);
//        sleep(intervalWait);
//
//        while (triggerMotor.getCurrentPosition() > targetPosition) {
//            triggerMotor.setPower(-1);
//        }
//        triggerMotor.setPower(0);
//        sleep(holdWait);
//        while (triggerMotor.getCurrentPosition() < 0) {
//            triggerMotor.setPower(1);
//        }
//        triggerMotor.setPower(0);
//        sleep(intervalWait);
//
//        while (triggerMotor.getCurrentPosition() > targetPosition) {
//            triggerMotor.setPower(-1);
//        }
//        triggerMotor.setPower(0);
//        sleep(holdWait);
//        while (triggerMotor.getCurrentPosition() < 0) {
//            triggerMotor.setPower(1);
//        }
//        triggerMotor.setPower(0);
        sleep(500);
        motor.setPower(0);
        motor2.setPower(0);
    }
}
