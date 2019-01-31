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

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="5518 TeleOp v3.0", group="TeleOp")
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

    private boolean isOnA            = false;
    private double slowModeValue     = 1.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables
        leftFrontDrive      = hardwareMap.get(DcMotor.class, "1");
        rightFrontDrive     = hardwareMap.get(DcMotor.class, "2");
        leftBackDrive       = hardwareMap.get(DcMotor.class, "3");
        rightBackDrive      = hardwareMap.get(DcMotor.class, "0");

        collector           = hardwareMap.get(DcMotor.class, "collector");
        spool               = hardwareMap.get(DcMotor.class, "spool");
        arm                 = hardwareMap.get(DcMotor.class, "arm");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        collector.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        spool.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        // Setup a variable for each drive wheel to save power level for telemetry



        if(gamepad1.a && !isOnA) {
            if(slowModeValue == 1.0)
                slowModeValue = 0.5;
            else
                slowModeValue = 1.0;
            isOnA = true;
        }
        else if(!gamepad1.a)
            isOnA = false;

        if(gamepad2.right_bumper)
        {
            collector.setPower(0.5);
        }
        else if (gamepad2.left_bumper)
        {
            collector.setPower(-0.5);
        }
        else
        {
            collector.setPower(0);
        }

        spool.setPower(gamepad2.right_stick_x);
        arm.setPower(gamepad2.left_stick_y * 0.85);


        // Code for Split-Arcade Driver Control
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x; //added negitive

        //Calculations for motor power
        double r = Math.hypot(leftStickX, -leftStickY);
        double robotAngle = Math.atan2(-leftStickY, leftStickX) - Math.PI / 4;
        double rotate = rightStickX;
        final double frontLeft = r * Math.cos(robotAngle) + rotate;
        final double frontRight = r * Math.sin(robotAngle) - rotate;
        final double rearLeft = r * Math.sin(robotAngle) + rotate;
        final double rearRight = r * Math.cos(robotAngle) - rotate;

        if(leftStickX != 0 || leftStickY != 0 || rightStickX != 0) {
            if (leftStickX > 0.1 || leftStickX < -0.1) {
                //Motor Power Sets
                leftFrontDrive.setPower(frontLeft);
                rightFrontDrive.setPower(frontRight);
                leftBackDrive.setPower(rearLeft);
                rightBackDrive.setPower(rearRight);
            } else {
                //Motor Power Sets
                leftFrontDrive.setPower(frontLeft * slowModeValue);
                rightFrontDrive.setPower(frontRight * slowModeValue);
                leftBackDrive.setPower(rearLeft * slowModeValue);
                rightBackDrive.setPower(rearRight * slowModeValue);
            }
        }
        else
        {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Slow-mode: ", isOnA);
        telemetry.addData("Front Motors", "leftFront (%.2f), rightFront (%.2f), ", frontLeft, frontRight);
        telemetry.addData("Back Motors", "leftBack (%.2f), rightBack (%.2f), ", rearLeft, rearRight);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
