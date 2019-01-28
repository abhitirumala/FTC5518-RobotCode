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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Locale;

/**
 * File Created bu Abhiram T. (5518)
 * Version 3.0
 * Date: 1/23/2018
 *
 *
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Silver-Side Autonomous", group="Autonomous")
//@Disabled
public class SilverAutonomousGyroEncoder extends LinearOpMode
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Motors
    private DcMotor leftFrontDrive   = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor leftBackDrive    = null;
    private DcMotor rightBackDrive   = null;

    private DcMotor arm              = null;
    private DcMotor spool            = null;

    //Gyro
    private BNO055IMU imu;
    private Orientation angles;

    //DogeCV Alignment Detector
    private GoldAlignDetector detector;

    private static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: AndyMark Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables
        leftFrontDrive      = hardwareMap.get(DcMotor.class, "1");
        rightFrontDrive     = hardwareMap.get(DcMotor.class, "2");
        leftBackDrive       = hardwareMap.get(DcMotor.class, "3");
        rightBackDrive      = hardwareMap.get(DcMotor.class, "0");

        arm                 = hardwareMap.get(DcMotor.class, "arm");
        spool               = hardwareMap.get(DcMotor.class, "spool");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        arm.setDirection(DcMotor.Direction.FORWARD);
        spool.setDirection(DcMotor.Direction.FORWARD);
        initializeGyro();

        initializeGoldAlignDetector();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        encoderDrive(0.5, 1.5, 1.5, 10);

        gyroTurn(-60, 0.2, true);




        while(!detector.getAligned())
        {
            leftFrontDrive.setPower(-0.1);
            rightFrontDrive.setPower(0.1);
            leftBackDrive.setPower(-0.1);
            rightBackDrive.setPower(0.1);
        }

        timedTurn(0.5, true);
        sleep (400);
        timedTurn(0, true);

        encoderDrive(0.75, 36, 36, 10);
        encoderDrive(0.5, -6, -6, 10);


        if (angles.firstAngle > 0)
            gyroTurn(8, 0.07, true);
        else
            gyroTurn(-8, 0.07, false);

        encoderDrive(1.0, 30, 30, 10);
    }


    private void powerMotors(double power, int timeInMS)
    {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep(timeInMS);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    private void timedTurn(double power, boolean isRight) {
        if (!isRight) {
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
        } else {
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
        }
    }

    private void initializeGyro()
    {
        /*
         * Save GyroSensor Calibration Data
         */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
        String filename = "AdafruitIMU1Calibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
        telemetry.log().add("saved to '%s'", filename);

        //Read Calibration Data
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        composeGyroTelemetry();
    }

    private void gyroTurn(double targetAngle, double turnPower, boolean isRight) {

        if (!isRight) {
            telemetry.update();
            double turnSpeedMultiplier;

            while ((Math.abs(angles.firstAngle - targetAngle) > 6) && opModeIsActive()) {
                telemetry.update();
                //turnSpeedMultiplier = (Math.toRadians(targetAngle - angles.firstAngle) * 0.5) + 0.35;
                turnSpeedMultiplier = 1;

                leftFrontDrive.setPower(-turnPower * turnSpeedMultiplier);
                rightFrontDrive.setPower(turnPower * turnSpeedMultiplier);
                leftBackDrive.setPower(-turnPower * turnSpeedMultiplier);
                rightBackDrive.setPower(turnPower * turnSpeedMultiplier);
            }
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            sleep(500);
        } else {
            telemetry.update();
            //targetAngle = -targetAngle;
            double turnSpeedMultiplier;

            while ((Math.abs(angles.firstAngle - targetAngle) > 6) && opModeIsActive()) {
                telemetry.update();
                //turnSpeedMultiplier = (Math.toRadians(targetAngle - angles.firstAngle) * 0.5) + 0.40;
                turnSpeedMultiplier = 1;

                leftFrontDrive.setPower(turnPower * turnSpeedMultiplier);
                rightFrontDrive.setPower(-turnPower * turnSpeedMultiplier);
                leftBackDrive.setPower(turnPower * turnSpeedMultiplier);
                rightBackDrive.setPower(-turnPower * turnSpeedMultiplier);
            }
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            sleep(500);
        }
    }

    private void initializeGoldAlignDetector()
    {
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1, false); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!
    }

    private void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS)
    {
        int newFrontLeftTarget;
        int newRearLeftTarget;
        int newFrontRightTarget;
        int newRearRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRearLeftTarget = leftBackDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRearRightTarget = rightBackDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newFrontLeftTarget);
            leftBackDrive.setTargetPosition(newRearLeftTarget);
            rightFrontDrive.setTargetPosition(newFrontRightTarget);
            rightBackDrive.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget,
                        newRearLeftTarget, newRearRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);
        }
    }

    private void composeGyroTelemetry()
    {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });
        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
    }
    private String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    private String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
