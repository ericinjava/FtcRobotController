package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous
public class Multithreading extends LinearOpMode {
    // The IMU sensor object
    BNO055IMU imu;
    NormalizedColorSensor colorSensor;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //set Motors
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    public Multithreading() throws Exception {
        telemetry.addData("Starting:", "New Thread");
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //hardware Mapping Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        //reversing Motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        final float[] hsvValues = new float[3];
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        // create an instance of the DriveThread.

        Thread driveThread = new DriveThread();

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        telemetry.addData("wait for start", "");

        waitForStart();


        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        telemetry.addData("started", "");

        // start the driving thread.
        //Starts using Gyro
        driveThread.start();

        // continue with main thread.
        //main thread runs color sensor
        try {
            while (opModeIsActive()) {
                telemetry.addData("Mode", "running");
                telemetry.addData("Run Time", this.getRuntime());
                telemetry.update();

                boolean sensedWhite = false;
                boolean sensedRed = false;
                while (!sensedWhite && opModeIsActive()) {
                    NormalizedRGBA colors = colorSensor.getNormalizedColors();

                    //Color.colorToHSV(colors.toColor(), hsvValues);


                    telemetry.addLine()
                            .addData("Red", "%.3f", colors.red * 100)
                            .addData("Green", "%.3f", colors.green * 100)
                            .addData("Blue", "%.3f", colors.blue * 100);
                    telemetry.addLine();
                    telemetry.addData("Alpha", "%.3f", colors.alpha);


                    telemetry.update();


                    leftFront.setPower(0.25);
                    rightFront.setPower(0.25);
                    leftBack.setPower(0.25);
                    rightBack.setPower(0.25);

                    if (colors.alpha > 0.3) {
                        sensedWhite = true;
                        leftFront.setPower(0);
                        rightFront.setPower(0);
                        leftBack.setPower(0);
                        rightBack.setPower(0);
                        sleep(1000);
                    }

                }
                for (int i = 0; i < 2; i++) {
                    while (!sensedRed && opModeIsActive()) {
                        NormalizedRGBA colors = colorSensor.getNormalizedColors();

                        //Color.colorToHSV(colors.toColor(), hsvValues);


                        telemetry.addLine()
                                .addData("Red", "%.3f", colors.red * 100)
                                .addData("Green", "%.3f", colors.green * 100)
                                .addData("Blue", "%.3f", colors.blue * 100);
                        telemetry.addLine();
                        telemetry.addData("Alpha", "%.3f", colors.alpha);


                        telemetry.update();


                        leftFront.setPower(0.25);
                        rightFront.setPower(0.25);
                        leftBack.setPower(0.25);
                        rightBack.setPower(0.25);

                        if (colors.alpha < 0.1) {
                            sensedRed = true;
                            leftFront.setPower(0);
                            rightFront.setPower(0);
                            leftBack.setPower(0);
                            rightBack.setPower(0);
                            sleep(1000);
                        }
                    }
                    sensedRed = false;
                }
            }

            idle();

        } catch (Exception e) {
            telemetry.addData(e.getMessage(), "");
        }

        telemetry.addData("out of while loop", "");

        // stop the driving thread.

        driveThread.interrupt();

        telemetry.addData("end", "");
    }

    private class DriveThread extends Thread {
        public DriveThread() {
            this.setName("DriveThread");

            telemetry.addData("%s", this.getName());
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            telemetry.addData("Starting thread %s", this.getName());

            try {
                while (!isInterrupted()) {
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.
                    leftFront.setPower(1);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                    double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                    double speed = 0.35; //Values betwen 0.25 and 0.5 work best
                    double proportionalTerm = 0.025; // Do not use any value above 0.025

                    if (heading < -0.1 && heading > -90) {
                        leftFront.setPower(speed - (proportionalTerm * heading));
                        leftBack.setPower(speed - (proportionalTerm * heading));
                        rightFront.setPower(speed + (proportionalTerm * heading));
                        rightBack.setPower(speed + (proportionalTerm * heading));
                    } else if (heading > 0.1 && heading < 90) {
                        leftFront.setPower(speed + (proportionalTerm * heading));
                        leftBack.setPower(speed + (proportionalTerm * heading));
                        rightFront.setPower(speed - (proportionalTerm * heading));
                        rightBack.setPower(speed - (proportionalTerm * heading));
                    } else {
                        leftFront.setPower(speed);
                        leftBack.setPower(speed);
                        rightFront.setPower(speed);
                        rightBack.setPower(speed);

                    }
                    telemetry.update();

                    idle();
                }
            } catch (Exception e) {
                telemetry.addData("u", "bad");
            }

            telemetry.addData("end of thread %s", this.getName());
        }


        String formatAngle(AngleUnit angleUnit, double angle) {
            return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }

        String formatDegrees(double degrees) {
            return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }
    }
}