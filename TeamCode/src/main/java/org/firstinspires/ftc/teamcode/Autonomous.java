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

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.util.Locale;

/**
 * File Created bu Abhiram T. (5518)
 * Version 1.0
 * Date: 1/23/2018
 *
 * Contents:
 * Write premature Autonomous to initialize basic operations
 */

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Blue Crater 1.0", group="Autonomous")
=======
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous FINAL", group="Autonomous")
>>>>>>> ab7d22d178ea757560e1a3e2458215bf1e7575ed
=======
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous FINAL", group="Autonomous")
>>>>>>> ab7d22d178ea757560e1a3e2458215bf1e7575ed
=======
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Sampling Test 1.1", group="Autonomous")
>>>>>>> parent of ab7d22d... 1/19 changes
=======
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Sampling Test 1.1", group="Autonomous")
>>>>>>> parent of ab7d22d... 1/19 changes
//@Disabled
public class Autonomous extends LinearOpMode
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Motors
    private DcMotor leftFrontDrive   = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor leftBackDrive    = null;
    private DcMotor rightBackDrive   = null;

    private static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: AndyMark Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 0.6 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 5.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    //Gyro
    private BNO055IMU imu;
    private Orientation angles;

    //DogeCV Alignment Detector
    private GoldAlignDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "1");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "2");
        leftBackDrive = hardwareMap.get(DcMotor.class, "3");
        rightBackDrive = hardwareMap.get(DcMotor.class, "0");
<<<<<<< HEAD
<<<<<<< HEAD
        //mrGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        //gyro = (IntegratingGyroscope) mrGyro;
<<<<<<< HEAD


=======


>>>>>>> ab7d22d178ea757560e1a3e2458215bf1e7575ed
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
=======

=======

>>>>>>> parent of ab7d22d... 1/19 changes
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
<<<<<<< HEAD
>>>>>>> parent of ab7d22d... 1/19 changes
=======
>>>>>>> parent of ab7d22d... 1/19 changes

        /*
         * Save GyroSensor Calibration Data
         */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();

        String filename = "AdafruitIMUCalibration.json";
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

        // Set up detector
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



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);


        manualTurn(1, false);
        sleep (1000);
        manualTurn(0, false);

//        manualTurn(-1, false);
//        sleep (1000);
//        manualTurn(0, false);

        //turnToAngle(false, 50, 1);

        while(!detector.getAligned())
        {
            manualTurn(0.4, true);
        }

        powerMotors(-0.5, 1500);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> parent of ab7d22d... 1/19 changes
=======
>>>>>>> parent of ab7d22d... 1/19 changes
    }

    private void turnToAngle(boolean isRight, double targetAngle, double turnPower)
    {
        if (isRight)
        {
            telemetry.update();
<<<<<<< HEAD
<<<<<<< HEAD
            targetAngle = -targetAngle;

            while (!(angles.firstAngle < targetAngle + 5 && angles.firstAngle > targetAngle - 10) && opModeIsActive()) {
                telemetry.update();

                leftFrontDrive.setPower(turnPower);
                rightFrontDrive.setPower(-turnPower);
                leftBackDrive.setPower(turnPower);
                rightBackDrive.setPower(-turnPower);
            }

            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
        else
        {
            telemetry.update();

            while (!(angles.firstAngle < targetAngle + 5 && angles.firstAngle > targetAngle - 10) && opModeIsActive()) {
                telemetry.update();

                leftFrontDrive.setPower(-turnPower);
                rightFrontDrive.setPower(turnPower);
                leftBackDrive.setPower(-turnPower);
                rightBackDrive.setPower(turnPower);
            }

            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
=======
        manualTurn(0.4, false);
        sleep ((int) angleTime*1000);
        manualTurn(0, false);
=======
>>>>>>> parent of ab7d22d... 1/19 changes

            double turnSpeedMultiplier;

            while ((Math.abs(angles.firstAngle - targetAngle) > 5) && opModeIsActive()) {
                telemetry.update();
                turnSpeedMultiplier = (Math.toRadians(targetAngle - angles.firstAngle) * 0.5) + 0.35;

<<<<<<< HEAD
        powerMotors(1, 5000);
=======
        manualTurn(0.4, false);
        sleep ((int) angleTime*1000);
        manualTurn(0, false);
=======
>>>>>>> parent of ab7d22d... 1/19 changes

            double turnSpeedMultiplier;

            while ((Math.abs(angles.firstAngle - targetAngle) > 5) && opModeIsActive()) {
                telemetry.update();
                turnSpeedMultiplier = (Math.toRadians(targetAngle - angles.firstAngle) * 0.5) + 0.35;

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
        else
        {
            telemetry.update();
            targetAngle = -targetAngle;
            double turnSpeedMultiplier;

            while ((Math.abs(angles.firstAngle - targetAngle) > 1) && opModeIsActive()) {
                telemetry.update();
                turnSpeedMultiplier = (Math.toRadians(targetAngle - angles.firstAngle) * 0.5) + 0.40;

                leftFrontDrive.setPower(turnPower * turnSpeedMultiplier);
                rightFrontDrive.setPower(-turnPower * turnSpeedMultiplier);
                leftBackDrive.setPower(turnPower * turnSpeedMultiplier);
                rightBackDrive.setPower(-turnPower * turnSpeedMultiplier);
            }

            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

<<<<<<< HEAD
>>>>>>> ab7d22d178ea757560e1a3e2458215bf1e7575ed


>>>>>>> ab7d22d178ea757560e1a3e2458215bf1e7575ed
=======
            sleep(500);
        }
>>>>>>> parent of ab7d22d... 1/19 changes

=======
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
        else
        {
            telemetry.update();
            targetAngle = -targetAngle;
            double turnSpeedMultiplier;

            while ((Math.abs(angles.firstAngle - targetAngle) > 1) && opModeIsActive()) {
                telemetry.update();
                turnSpeedMultiplier = (Math.toRadians(targetAngle - angles.firstAngle) * 0.5) + 0.40;

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

>>>>>>> parent of ab7d22d... 1/19 changes
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

    private void manualTurn(double power, boolean isRight)
    {
        if (isRight)
        {
<<<<<<< HEAD
<<<<<<< HEAD
=======
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
        }
        else
        {
>>>>>>> parent of ab7d22d... 1/19 changes
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
        }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    }

    private void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
=======
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
>>>>>>> parent of ab7d22d... 1/19 changes
    }

    private String formatAngle(AngleUnit angleUnit, double angle)
    {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

<<<<<<< HEAD
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
=======
        else
>>>>>>> ab7d22d178ea757560e1a3e2458215bf1e7575ed
=======
        else
>>>>>>> ab7d22d178ea757560e1a3e2458215bf1e7575ed
        {
=======
>>>>>>> parent of ab7d22d... 1/19 changes
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);
        }
        else
        {
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
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

=======
>>>>>>> parent of ab7d22d... 1/19 changes
    private String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
