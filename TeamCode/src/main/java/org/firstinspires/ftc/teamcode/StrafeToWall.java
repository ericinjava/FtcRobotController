package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import java.util.Locale;

public class StrafeToWall extends LinearOpMode {
    // The IMU sensor object
    BNO055IMU imu;
    NormalizedColorSensor colorSensor;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //motors
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;

    private DistanceSensor sensorRange;
    private double distanceReading;


    @Override
    public void runOpMode() {

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensorRange");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //initializing motors
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftBack = hardwareMap.dcMotor.get("leftBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);    //Reverse
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);     //Reverse

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive()) {
            distanceReading = sensorRange.getDistance(DistanceUnit.CM);

            // Loop and update the dashboard
            while (opModeIsActive() && distanceReading > 5) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                double speed = 0.35; //Values between 0.25 and 0.5 work best
                double proportionalTerm = 0.025; // Do not use any value above 0.025

                if (heading < -0.1 && heading > -90) {
                    leftFront.setPower((speed - (proportionalTerm * heading)));
                    leftBack.setPower(-(speed - (proportionalTerm * heading)));
                    rightFront.setPower(-(speed + (proportionalTerm * heading)));
                    rightBack.setPower((speed + (proportionalTerm * heading)));
                } else if (heading > 0.1 && heading < 90) {
                    leftFront.setPower((speed + (proportionalTerm * heading)));
                    leftBack.setPower(-(speed + (proportionalTerm * heading)));
                    rightFront.setPower(-(speed - (proportionalTerm * heading)));
                    rightBack.setPower((speed - (proportionalTerm * heading)));
                } else {
                    leftFront.setPower(speed);
                    leftBack.setPower(-speed);
                    rightFront.setPower(-speed);
                    rightBack.setPower(speed);

                }
                telemetry.update();
            }

        }
    }


    public void driveStraightFor(int duration, double power) throws InterruptedException {
        double leftSpeed;
        double rightSpeed;

        double target =
                leftFront.getCurrentPosition();
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}