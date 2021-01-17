package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.*;


@TeleOp(name = "Sensor: BNO055 IMU", group = "Sensor")
public class IMUTest extends LinearOpMode {

    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    @Override
    public void runOpMode() {


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");


        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Loop and update the dashboard
        while (opModeIsActive() && !isStopRequested()){
            driveStraight(1000, -0.9);
        }
    }

    public void turnTo(int targetHeading) throws InterruptedException{


        while (imu.isGyroCalibrated()) {
            double currentHeading = angles.firstAngle;
            //set Speed Of Turn
            double turnSpeed = 0.15;

            while (Math.abs(currentHeading - targetHeading) > 3) {
                if (currentHeading > targetHeading){
                    //turn Left
                    leftFront.setPower(-turnSpeed);
                    rightFront.setPower(turnSpeed);
                    leftBack.setPower(-turnSpeed);
                    rightBack.setPower(turnSpeed);
                }
                if (currentHeading < targetHeading){
                    //turnRight
                    leftFront.setPower(turnSpeed);
                    rightFront.setPower(-turnSpeed);
                    leftBack.setPower(turnSpeed);
                    rightBack.setPower(-turnSpeed);
                }

                currentHeading = angles.firstAngle;
                //telemetry update
                telemetry.addData("1. accu", String.format("803d", currentHeading));
            }

            //set Power to 0
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            //telemetry update and sleep
            telemetry.addData("1. accu", String.format("803d", currentHeading));
            sleep(25);
        }
    }

    public void driveStraight(int duration, double power) {
        angles = imu.getAngularOrientation(INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        telemetry.addData("Roll", angles.secondAngle);
        telemetry.addData("Pitch", angles.thirdAngle);
        telemetry.update();

        double leftSpeed;
        double rightSpeed;

        double target = angles.firstAngle;

        while (System.currentTimeMillis() < duration) {
            double absolute = angles.firstAngle;
            leftSpeed = power + (absolute - target)/100;
            rightSpeed = power - (absolute - target)/100;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            leftFront.setPower(leftSpeed);
            rightFront.setPower(rightSpeed);
            leftBack.setPower(leftSpeed);
            rightBack.setPower(rightSpeed);

        }
    }
}
