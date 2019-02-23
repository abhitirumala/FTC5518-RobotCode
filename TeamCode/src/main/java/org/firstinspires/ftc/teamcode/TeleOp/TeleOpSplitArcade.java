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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import java.util.concurrent.TimeUnit;

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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="5518 TeleOp v5.1", group="TeleOp")
//@Disabled
public class TeleOpSplitArcade extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime      = new ElapsedTime();

    private DcMotor leftFrontDrive   = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor leftBackDrive    = null;
    private DcMotor rightBackDrive   = null;
    private DcMotor collector        = null;
    private DcMotor spool            = null;
    private DcMotor arm              = null;
    private DcMotor winch            = null;



    private boolean isOnA            = false;
    private double slowModeValue     = 1.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {

        // Initialize the hardware variables
        leftFrontDrive      = hardwareMap.get(DcMotor.class, "1");
        rightFrontDrive     = hardwareMap.get(DcMotor.class, "2");
        leftBackDrive       = hardwareMap.get(DcMotor.class, "3");
        rightBackDrive      = hardwareMap.get(DcMotor.class, "0");

        collector           = hardwareMap.get(DcMotor.class, "collector");
        spool               = hardwareMap.get(DcMotor.class, "spool");
        arm                 = hardwareMap.get(DcMotor.class, "arm");
        winch               = hardwareMap.get(DcMotor.class, "winch");


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collector.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        spool.setDirection(DcMotor.Direction.FORWARD);
        winch.setDirection(DcMotor.Direction.FORWARD);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop()
    {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start()
    {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
//        double interationStart = runtime.now(TimeUnit.SECONDS);
        // Setup a variable for each drive wheel to save power level for telemetry
        //telemetry.addData("Reached start loop", "");


        if(gamepad1.a && !isOnA) {
            if(slowModeValue == 1.0)
                slowModeValue = 0.5;
            else
                slowModeValue = 1.0;
            isOnA = true;
        }
        else if(!gamepad1.a)
            isOnA = false;

        //telemetry.addData("Reached slowmode", "");

        if(gamepad2.right_bumper)
        {
            collector.setPower(0.8);
        }
        else if (gamepad2.left_bumper)
        {
            collector.setPower(-0.8);
        }
        else
        {
            collector.setPower(0);
        }

        //telemetry.addData("Reached collector", "");

        if (gamepad1.right_bumper)
            winch.setPower(1);
        else if (gamepad1.left_bumper)
            winch.setPower(-1);
        else
            winch.setPower(0);

        //telemetry.addData("Reached winch", "");



        spool.setPower(gamepad2.right_stick_x);

        //telemetry.addData("Reached spool", "");

        arm.setPower(gamepad2.left_stick_y * 0.75);

        //telemetry.addData("Reached arm", "");


        double forward = gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double clockwise = -gamepad1.right_stick_x;

        double v1 = Range.clip(forward + clockwise - right, -1.0, 1.0);
        double v2 = Range.clip(forward - clockwise - right, -1.0, 1.0);
        double v3 = Range.clip(forward + clockwise + right, -1.0, 1.0);
        double v4 = Range.clip(forward - clockwise + right, -1.0, 1.0);

        leftFrontDrive.setPower(v1 * slowModeValue);
        rightFrontDrive.setPower(v2 * slowModeValue);
        leftBackDrive.setPower(v3 * slowModeValue);
        rightBackDrive.setPower(v4 * slowModeValue);

        //telemetry.addData("Reached drive", "");


        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Status", "Iteration Loop Time: " + (runtime.now(TimeUnit.SECONDS) - interationStart));
        telemetry.addData("Slow-mode: ", isOnA);
        telemetry.addData("Calculated Front Motors: ", "leftFront (%.2f), rightFront (%.2f), ", v1, v2);
        telemetry.addData("Calculated Back Motors: ", "leftBack (%.2f), rightBack (%.2f), ", v3, v4);

        /*telemetry.addData("Actual Front Motors: ", "leftFront (%.2f), rightFront (%.2f), ", leftFrontDrive.getPower(), rightFrontDrive.getPower());
        telemetry.addData("Actual Back Motors: ", "leftBack (%.2f), rightBack (%.2f), ", leftBackDrive.getPower(), rightBackDrive.getPower());
        telemetry.addData("Arm: ", arm.getCurrentPosition());*/

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
