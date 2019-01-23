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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.io.File;
import java.util.Locale;
/**
 * File Created bu Abhiram T. (5518)
 * Version 1.2
 * Date: 9/15/2018
 *
 * Contents:
 * Perform Simple Robot Maneuvers using the Adafuit REV Expansion Hub Built-in Gyro Sensor
 * Maneuver: Robot drives in a square, turning after every second of travel
 */
@Autonomous(name="Autonomous Square 1.0", group="Autonomous")
@Disabled
public class AutoGyroTankDriveTest extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive  = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private static final double     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: AndyMark Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 0.6 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 5.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private BNO055IMU imu;
    private Orientation angles;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize the hardware variables
        frontLeftDrive = hardwareMap.get(DcMotor.class, "1");
        frontRightDrive = hardwareMap.get(DcMotor.class, "2");
        backLeftDrive = hardwareMap.get(DcMotor.class, "3");
        backRightDrive = hardwareMap.get(DcMotor.class, "0");
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        resetEncoders();
        /*
         * Save GyroSensor Calibration Data
         */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu2 = hardwareMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters);
//        imu2.initialize(parameters);
        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
//        BNO055IMU.CalibrationData calibrationData2 = imu2.readCalibrationData();
        String filename = "AdafruitIMU1Calibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
        telemetry.log().add("saved to '%s'", filename);
//        String filename2 = "AdafruitIMU2Calibration.json";
//        File file2 = AppUtil.getInstance().getSettingsFile(filename2);
//        ReadWriteFile.writeFile(file2, calibrationData2.serialize());
//        telemetry.log().add("saved to '%s'", filename2);
        //Read Calibration Data
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
//        imu2.initialize(parameters);
        composeGyroTelemetry();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        for (int i=0; i<4; i++)
        {
            encoderDrive(0.5, 5,-5,10);
            encoderDrive(0.75,12,12, 10);
        }
        for (int multiplier=1; multiplier<=4; multiplier++)
        {

            if (multiplier == 1)
                turnToAngle(true,90,1);
            else if (multiplier == 2)
                turnToAngle(true, 180,1);
            else if (multiplier == 3)
                turnToAngle(true, -90,1);
            else
                turnToAngle(true, 0, 1);

            encoderDrive(0.75,12,12, 10);
        }
        for (int multiplier=4; multiplier>=1; multiplier--)
        {
            if (multiplier == 1)
                turnToAngle(true,90,1);
            else if (multiplier == 2)
                turnToAngle(true, 180,1);
            else if (multiplier == 3)
                turnToAngle(true, -90,1);
            else
                turnToAngle(true, 0, 1);
            powerMotors(0.5);
            sleep(1000);
            powerMotors(0.0);
        }
        encoderDrive(0.75,12,12, 10);
    }
    // Resets Encoder Positions
    private void resetEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS)
    {
        int newFrontLeftTarget;
        int newRearLeftTarget;
        int newFrontRightTarget;
        int newRearRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRearLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRearRightTarget = backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            backLeftDrive.setTargetPosition(newRearLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backRightDrive.setTargetPosition(newRearRightTarget);
            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && backLeftDrive.isBusy() && backRightDrive.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget,
                        newRearLeftTarget, newRearRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        frontLeftDrive.getCurrentPosition(),
                        frontRightDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(),
                        backRightDrive.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            frontLeftDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backRightDrive.setPower(0);
            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    private void turnToAngle(boolean isRight, double targetAngle, double turnPower)
    {
        if (isRight)
        {
            telemetry.update();
            double turnSpeedMultiplier;

            while ((Math.abs(angles.firstAngle - targetAngle) > 5) && opModeIsActive()) {
                telemetry.update();
                //turnSpeedMultiplier = (Math.toRadians(targetAngle - angles.firstAngle) * 0.5) + 0.35;
                turnSpeedMultiplier=1;

                frontLeftDrive.setPower(turnPower * turnSpeedMultiplier);
                frontRightDrive.setPower(-turnPower * turnSpeedMultiplier);
                backLeftDrive.setPower(turnPower * turnSpeedMultiplier);
                backRightDrive.setPower(-turnPower * turnSpeedMultiplier);
            }
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            sleep(500);
        }
        else
        {
            telemetry.update();
            targetAngle = -targetAngle;
            double turnSpeedMultiplier;

            while ((Math.abs(angles.firstAngle - targetAngle) > 1) && opModeIsActive()) {
                telemetry.update();
                //turnSpeedMultiplier = (Math.toRadians(targetAngle - angles.firstAngle) * 0.5) + 0.40;
                turnSpeedMultiplier=1;

                frontLeftDrive.setPower(turnPower * turnSpeedMultiplier);
                frontRightDrive.setPower(-turnPower * turnSpeedMultiplier);
                backLeftDrive.setPower(turnPower * turnSpeedMultiplier);
                backRightDrive.setPower(-turnPower * turnSpeedMultiplier);
            }
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            sleep(500);
        }
    }
    private void powerMotors(double power)
    {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
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
