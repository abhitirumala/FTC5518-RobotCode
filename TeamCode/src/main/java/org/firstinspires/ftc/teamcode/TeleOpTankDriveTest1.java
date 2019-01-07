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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * File Created by Abhiram T. (5518)
 * Version 1.1.1
 * Date: 11/2/2018
 *
 * Contents:
 * Split Arcade Style drive control with linear movement and rotation
 * Ability to change the speed multiplier from 1.0 to 0.5
 * arm, collector, and spool code
 */

@TeleOp(name="TeleOp Tank Drive Test1", group="Tele-Op")
//@Disabled
public class TeleOpTankDriveTest1 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime      = new ElapsedTime();

    //Motors
    private DcMotor leftFrontDrive   = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor leftBackDrive    = null;
    private DcMotor rightBackDrive   = null;
    private DcMotor armPivot         = null;
    private DcMotor spool            = null;
    private DcMotor collector        = null;

    private double slowModeValue = 1.0;
    private boolean isOnA = false;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "2");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "3");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "0");

        armPivot        = hardwareMap.get(DcMotor.class, "arm");
        spool           = hardwareMap.get(DcMotor.class, "spool");
        collector       = hardwareMap.get(DcMotor.class, "collector");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        armPivot.setDirection(DcMotor.Direction.FORWARD);
        spool.setDirection(DcMotor.Direction.FORWARD);
        collector.setDirection(DcMotor.Direction.FORWARD);

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
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        //Slow-Mode - Switch between 100% and 50%
        if(gamepad1.a && !isOnA)
        {
            if(slowModeValue == 1.0)
                slowModeValue = 0.5;
            else
                slowModeValue = 1.0;
            isOnA = true;
        }
        else if(!gamepad1.a)
            isOnA = false;

        // Code for Split-Arcade Driver Control
        double drive =  gamepad1.left_stick_y * slowModeValue;
        double turn  =  -gamepad1.right_stick_x;
        leftFrontPower    = Range.clip(drive + turn, -1.0, 1.0);
        rightFrontPower   = Range.clip(drive - turn, -1.0, 1.0);
        leftBackPower     = Range.clip(drive + turn, -1.0, 1.0);
        rightBackPower    = Range.clip(drive - turn, -1.0, 1.0);

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        //Code for arm, collector and spool - arm is on right stick and spool is on left (for second controller)
        double armPower = gamepad2.right_stick_y;
        if (gamepad2.right_bumper)
            collector.setPower(1.0);
        else if (gamepad2.left_bumper)
            collector.setPower(-0.5);
        else
            collector.setPower(0.0);

        if (gamepad1.dpad_up)
            spool.setPower(1);
        else if (gamepad1.dpad_down)
            spool.setPower(-1);
        else
            spool.setPower(0);

        //TELEMETRY CODE
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front Motors", "leftFront (%.2f), rightFront (%.2f), ", leftFrontPower, rightFrontPower);
        telemetry.addData("Back Motors", "leftBack (%.2f), rightBack (%.2f), ", leftBackPower, rightBackPower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
