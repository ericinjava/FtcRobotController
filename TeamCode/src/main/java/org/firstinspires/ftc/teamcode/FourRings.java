package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

public class FourRings extends LinearOpMode {

    // Declaring Motors
    private     DcMotorEx     rf;         //port 0
    private     DcMotorEx     lf;         //port 1
    private     DcMotorEx     rb;         //port 2
    private     DcMotorEx     lb;         //port 3
    private     DcMotor     rs;         //port 0
    private     DcMotor     rl1;        //port 1
    private     DcMotor     rl2;        //port 2
    private     DcMotor     as;         //port 3

    private Servo _rf;        //port 0
    private     Servo       ts;         //port 0
    private     Servo       bs;         //port 1

    NormalizedColorSensor colorSensor;
    boolean foundRed = false;
    boolean foundWhite = false;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    boolean turned180 = false;
    boolean didStrafe = false;

    private DistanceSensor sensorRange;
    private double distanceReading;

    //Declaring Camera Variables
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;
    private static final String LABEL_FIRST_ELEMENT = "Quad Stack";
    private static final String LABEL_SECOND_ELEMENT = "Single Stack";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String VUFORIA_KEY = "ARxaOAX/////AAABmR91q9ci+kNYqGb/NElhuhBQa5klidYZ5jKk5hFYJ6qAQOtCGKSEZXn1qYawipXKEEpJh+vP3GNnOUvabO2blz4vkymDnu8LUocLc6/rMpQdLwBt80JVdgWWkd/4j1DmwDdRRP4f/jP78furjgexjT7HgmC37xLP+msr78zAeWwkrsT2X1yjnL6nyiGcRKlBw6+EcUIcZYiiuXwbILds8rl4Fu7AuecLaygDft6XIUFg/qQm51UF45l5pYT8AoNTUhP9GTksKkmHgde7iGlo3CfIYu9QanjPHreT/+JZLJWG22jWC7Nnzch/1HC6s3s2jzkrFV6sRVA4lL9COLIonjRBYPhbxCF06c5fUMy9sj/e";

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Stat", "Initializing...");
        telemetry.update();

        // Mapping Devices
        rf  =   hardwareMap.get(DcMotorEx.class, "rightFront");
        lf  =   hardwareMap.get(DcMotorEx.class, "leftFront");
        rb  =   hardwareMap.get(DcMotorEx.class, "rightBack");
        lb  =   hardwareMap.get(DcMotorEx.class, "leftBack");

        rs  =   hardwareMap.dcMotor.get("ringScoop");
        rl1 =   hardwareMap.dcMotor.get("ringLaunch1");
        rl2 =   hardwareMap.dcMotor.get("ringLaunch2");
        as  =   hardwareMap.dcMotor.get("armString");

        _rf =   hardwareMap.servo.get("ringFling");
        ts  =   hardwareMap.servo.get("topServo");
        bs  =   hardwareMap.servo.get("bottomServo");

        // Extra Motor Steps
        lf.setDirection(DcMotorSimple.Direction.REVERSE);    //Reverse
        lb.setDirection(DcMotorSimple.Direction.REVERSE);    //Reverse

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensorColor");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize Camera
        //initVuforia();
        //initTfod();

        if (tfod != null) {
            tfod.activate();
        }

        //Camera Detection
        ts.setPosition(0.92);
        bs.setPosition(0);
        _rf.setPosition(0.8);
        sleep(200);


        boolean singleStack = false;
        boolean quadStack = false;
        while (!opModeIsActive()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# of Rings", updatedRecognitions.size()); //# Object Detected
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("Label"), recognition.getLabel());
                        if(recognition.getLabel() == LABEL_SECOND_ELEMENT) {
                            singleStack = true;
                        } else if(recognition.getLabel() == LABEL_FIRST_ELEMENT) {
                            quadStack = true;
                        }
                    }
                    telemetry.update();
                }
            }
        }
        imu.initialize(parameters);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Stat", "Start Program");
        telemetry.update();

        //Waiting for start via Player
        waitForStart();

        senseLine("white", 0.35);
        strafeRightUntil(30);
        senseLine("red", 0.35);
        senseLine("red", 0.35);

        moveInches(8);
        turn(0.35, 120);
        moveInches(-2);
        wobbleDrop();
        moveInches(2);
        turn(-0.35, 120);
        senseLine("red", -0.35);

        strafeRightUntil(5);
        senseLineFollowWall("white");
    }

    void strafeRightUntil(double centimetersFromWall) {

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive()) {
            distanceReading = sensorRange.getDistance(DistanceUnit.CM);

            // Loop and update the dashboard
            while (opModeIsActive() && distanceReading > centimetersFromWall) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                double speed = 0.35; //Values between 0.25 and 0.5 work best
                double proportionalTerm = 0.025; // Do not use any value above 0.025

                if (heading < -0.1 && heading > -90) {
                    lf.setPower((speed - (proportionalTerm * heading)));
                    lb.setPower(-(speed + (proportionalTerm * heading)));
                    rf.setPower(-(speed + (proportionalTerm * heading)));
                    rb.setPower((speed + (proportionalTerm * heading)));
                } else if (heading > 0.1 && heading < 90) {
                    lf.setPower((speed + (proportionalTerm * heading)));
                    lb.setPower(-(speed + (proportionalTerm * heading)));
                    rf.setPower(-(speed - (proportionalTerm * heading)));
                    rb.setPower((speed - (proportionalTerm * heading)));
                } else {
                    lf.setPower(speed);
                    lb.setPower(-speed);
                    rf.setPower(-speed);
                    rb.setPower(speed);

                }
                telemetry.update();
            }

        }
    }

    //problems? see if-else block
    void senseLine(String color, double speed) {
        //Needed (non-changing) Variables
        final float[] hsvValues = new float[3];
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int countRed = 0;
        int countWhite = 0;

        //If the "foundRed" Boolean is False, Run Loop
        while (!foundRed && opModeIsActive()) {
            //Needed (updating) Variables
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            //If-Else-If Statement to Drive Forward in a Straight Line
            if (speed > 0) {
                if (heading < -0.1  && heading > -90){
                    lf.setPower(speed - (0.025 * heading));
                    lb.setPower(speed - (0.025 * heading));
                    rf.setPower(speed + (0.025 * heading));
                    rb.setPower(speed + (0.025 * heading));
                }else if (heading > 0.1 && heading < 90){
                    lf.setPower(speed + (0.025 * heading));
                    lb.setPower(speed + (0.025 * heading));
                    rf.setPower(speed - (0.025 * heading));
                    rb.setPower(speed - (0.025 * heading));
                } else {
                    lf.setPower(speed);
                    lb.setPower(speed);
                    rf.setPower(speed);
                    rb.setPower(speed);
                }
            } else if (speed < 0) {
                //May need to switch the positives and negatives if gyro seems not too work
                if (heading < -0.1  && heading > -90){
                    lf.setPower(speed - (0.025 * heading));
                    lb.setPower(speed - (0.025 * heading));
                    rf.setPower(speed + (0.025 * heading));
                    rb.setPower(speed + (0.025 * heading));
                }else if (heading > 0.1 && heading < 90){
                    lf.setPower(speed + (0.025 * heading));
                    lb.setPower(speed + (0.025 * heading));
                    rf.setPower(speed - (0.025 * heading));
                    rb.setPower(speed - (0.025 * heading));
                } else {
                    lf.setPower(speed);
                    lb.setPower(speed);
                    rf.setPower(speed);
                    rb.setPower(speed);
                }
            }

            //Telemetry Info for Diagnostics
            telemetry.addLine()
                    .addData("Alpha Output", "%.3f", colors.alpha)
                    .addData("Heading Output", "%.3f", heading)
                    .addData("Loop Count", "%,3f", countRed);
            telemetry.update();

            if (color == "red") {
                //If Statement to Detect the Red Line and Break the Loop
                if (colors.alpha < 0.2) {
                    stopRobot();
                    foundRed = true;
                }
                countRed++;
            } else if (color == "white") {
                if (colors.alpha > 0.5) {
                    stopRobot();
                    foundWhite = true;
                }
                countWhite++;
            } else {
                stopRobot();
            }
        }
        foundRed = false;
        foundWhite = false;
    }

    void moveInches(double lengthUsingInches) {
        double calcPosition = lengthUsingInches * (100* 280/(16.9646003294*4 *8.8 * 1.0555555556));
        int setPosition = (int) Math.round(calcPosition);

        lf.setTargetPosition(setPosition);
        rf.setTargetPosition(setPosition);
        lb.setTargetPosition(setPosition);
        rb.setTargetPosition(setPosition);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lf.setVelocity(500);
        rf.setVelocity(500);
        lb.setVelocity(500);
        rb.setVelocity(500);

        while (opModeIsActive() && lf.isBusy()) {
            telemetry.addData("position", lf.getCurrentPosition());
            telemetry.addData("is at target", !lf.isBusy());
            telemetry.update();
        }

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void turn(double speed, int angleMeasure) {
        while (opModeIsActive() && !turned180) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = Math.abs(Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)));

            telemetry.addData("heading", heading);
            telemetry.update();

            if (heading <= angleMeasure + 1 && heading >= angleMeasure - 1) {
                stopRobot();
                turned180 = true;
            } else if (heading >= (0.9 * angleMeasure) && heading < (angleMeasure - 1)) {
                rf.setPower(0.5 * speed);
                lf.setPower(-0.5 *speed);
                rb.setPower(0.5 * speed);
                lb.setPower(0.5 * -speed);
            } else {
                rf.setPower(speed);
                lf.setPower(-speed);
                rb.setPower(speed);
                lb.setPower(-speed);
            }
        }
    }

    void wobbleDrop() {
        bs.setPosition(0.9);
        sleep(100);
        ts.setPosition(0.5);
        sleep(25);
    }

    void senseLineFollowWall(String color/*, double speed, double centimetersFromWall*/) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        if (color == "white") {
            while (opModeIsActive() && !foundWhite) {
                if (colors.alpha > 0.5) {
                    stopRobot();
                    foundWhite = true;
                }
                lf.setPower((distanceReading/4) - 1.00);
                rf.setPower(-(distanceReading/4) + 1.50);
                lb.setPower((distanceReading/4) - 1.00);
                rb.setPower((-distanceReading/4) + 1.50);
            }
        } else if (color == "red") {
            while (opModeIsActive() && !foundRed) {
                if (colors.alpha < 0.2) {
                    stopRobot();
                    foundRed = true;
                }
                lf.setPower((distanceReading/4) - 1.00);
                rf.setPower(-(distanceReading/4) + 1.50);
                lb.setPower((distanceReading/4) - 1.00);
                rb.setPower((-distanceReading/4) + 1.50);
            }
        }
        foundRed = false;
        foundWhite = false;
    }

    //Method to Halt Robot Movement
    public void stopRobot() {
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


}
