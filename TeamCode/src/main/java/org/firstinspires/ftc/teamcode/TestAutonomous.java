package org.firstinspires.ftc.teamcode;

import java.util.List;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.Locale;
import java.util.Stack;
import android.view.View;
import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.Func;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


@Autonomous(name = "TestAutonomous", group = "comp")


/*******************************************************************************
 *                              COMP PROGRAM: START                             *
 *******************************************************************************/

// INITILIZATION
public class TestAutonomous extends LinearOpMode {

    // Declaring Motors
    private     DcMotorEx     rf;         //port 0
    private     DcMotorEx     lf;         //port 1
    private     DcMotorEx     rb;         //port 2
    private     DcMotorEx     lb;         //port 3
    private     DcMotor     rs;         //port 0
    private     DcMotor     rl1;        //port 1
    private     DcMotor     rl2;        //port 2
    private     DcMotor     as;         //port 3

    private     Servo       _rf;        //port 0
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


    //Declaring Camera Variables
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;
    private static final String LABEL_FIRST_ELEMENT = "Quad Stack";
    private static final String LABEL_SECOND_ELEMENT = "Single Stack";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String VUFORIA_KEY = "ARxaOAX/////AAABmR91q9ci+kNYqGb/NElhuhBQa5klidYZ5jKk5hFYJ6qAQOtCGKSEZXn1qYawipXKEEpJh+vP3GNnOUvabO2blz4vkymDnu8LUocLc6/rMpQdLwBt80JVdgWWkd/4j1DmwDdRRP4f/jP78furjgexjT7HgmC37xLP+msr78zAeWwkrsT2X1yjnL6nyiGcRKlBw6+EcUIcZYiiuXwbILds8rl4Fu7AuecLaygDft6XIUFg/qQm51UF45l5pYT8AoNTUhP9GTksKkmHgde7iGlo3CfIYu9QanjPHreT/+JZLJWG22jWC7Nnzch/1HC6s3s2jzkrFV6sRVA4lL9COLIonjRBYPhbxCF06c5fUMy9sj/e";

    @Override

    public void runOpMode() {
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
        initVuforia();
        initTfod();
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

        //Initializing Servos
        //bs.setPosition(0);
        //_rf.setPosition(1);

        //More telemetry data
        telemetry.addData("Status", "Pre-Coded Robot Driving");
        telemetry.update();

        //Beginning Loop for Program
        if (opModeIsActive()) {

            //as.setPower(1);
            //sleep(350);

            //shootRing(10);

            if(quadStack) {
                //wheelSpeed(1,1,1,1);
                sleep(1550);

                //wheelSpeed(0.5, -0.5, 0.5, -0.5);
                sleep(980);

                //wheelSpeed(-0.5, -0.5, -0.5, -0.5);
                sleep(1150);

                sleep(100);

                bs.setPosition(0.4);
                sleep(100);

                sleep(100);

                ts.setPosition(0.5);
                sleep(25);

                sleep(100);

                //wheelSpeed(0.5, -0.5, 0.5, -0.5);
                sleep(400);

                //wheelSpeed(1, 1, 1, 1);
                sleep(250);

            } else if(singleStack) {
                goToWhite();
                turn180();
                moveRobot(-0.25, -0.25, -0.25, -0.25, 100);
                stopRobot();
                bs.setPosition(0.5);
                sleep(50);
                ts.setPosition(0.1);
                sleep(50);
                //testForColor();

                /* wheelSpeed(1, 1, 1, 1);
                sleep(1500);

                wheelSpeed(0.5, -0.5, 0.5, -0.5);
                sleep(883);

                wheelSpeed(-0.5, -0.5, -0.5, -0.5);
                sleep(40);

                sleep(100);

                bs.setPosition(0.9);
                sleep(100);
                ts.setPosition(0.5);
                sleep(25);

                wheelSpeed(1, 1, 1, 1);
                sleep(90);

                wheelSpeed(0.5, -0.5, 0.5, -0.5);
                sleep(250);

                wheelSpeed(1, 1, 1, 1);
                sleep(6); */

            } else {
                strafe("r", 500, 36);
                //wheelSpeed(1, 1, 1, 1);
                /*sleep(1178);

                //wheelSpeed(0.5, -0.5, 0.5, -0.5);
                sleep(870);

                //wheelSpeed(-0.5, -0.5, -0.5, -0.5);
                sleep(1425);

                bs.setPosition(0.9);
                sleep(100);
                ts.setPosition(0.5);
                sleep(25);

                moveRobot(0.75, 0.75, 0.75, 0.75, 90); */
            }
            stopRobot();
        }
    }

    //Vufoira Method
    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }


    //Tfod Method
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }



    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    //Method to Completely Stop ALL Robot Movement
    private void forceStop() {
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
        as.setPower(0);
        rs.setPower(0);
        rl1.setPower(0);
        rl2.setPower(0);
        bs.setPosition(0);
        _rf.setPosition(0.8);
    }


    //Method to Halt Robot Movement
    public void stopRobot() {
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }


    //Method to Move Robot @ Designated Speed & Duration
    public void moveRobot(double rfSpeed, double lfSpeed, double rbSpeed, double lbSpeed, long dur) {
        rf.setPower(rfSpeed);
        lf.setPower(lfSpeed);
        rb.setPower(rbSpeed);
        lb.setPower(lbSpeed);
        sleep(dur);
    }


    //Method to Find & Move to the White Line
    public void goToWhite() {
        //Needed (non-changing) Variables
        final float[] hsvValues = new float[3];
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //If the "foundWhite" Boolean is False, Run Loop
        while (!foundWhite && opModeIsActive()) {
            //Needed (updating) Variables
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            int countWhite = 0;

            //If-Else-If Statment to Drive Forward in a Straight Line
            if (heading < -0.1  && heading > -90){
                lf.setPower(0.35 - (0.025 * heading));
                lb.setPower(0325 - (0.025 * heading));
                rf.setPower(0.35 + (0.025 * heading));
                rb.setPower(0.35 + (0.025 * heading));
            }else if (heading > 0.1 && heading < 90){
                lf.setPower(0.35 + (0.025 * heading));
                lb.setPower(0.35 + (0.025 * heading));
                rf.setPower(0.35 - (0.025 * heading));
                rb.setPower(0.35 - (0.025 * heading));
            } else {
                lf.setPower(0.35);
                lb.setPower(0.35);
                rf.setPower(0.35);
                rb.setPower(0.35);
            }

            //Telemetry Info for Diagnostics
            telemetry.addLine()
                    .addData("Alpha Output", "%.3f", colors.alpha)
                    .addData("Heading Output", "%.3f", heading)
                    .addData("Loop Count", countWhite);
            telemetry.update();

            //If Statement to Detect the White Line and Break the Loop
            if (colors.alpha > 0.5) {
                stopRobot();
                foundWhite = true;
            }
            countWhite++;
        }
    }


    //Method to find & Move to the Red line
    public void goToRed() {
        //Needed (non-changing) Variables
        final float[] hsvValues = new float[3];
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int countRed = 0;

        //If the "foundRed" Boolean is False, Run Loop
        while (!foundRed && opModeIsActive()) {
            //Needed (updating) Variables
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            //If-Else-If Statement to Drive Forward in a Straight Line
            if (heading < -0.1  && heading > -90){
                lf.setPower(0.25 - (0.025 * heading));
                lb.setPower(0.25 - (0.025 * heading));
                rf.setPower(0.25 + (0.025 * heading));
                rb.setPower(0.25 + (0.025 * heading));
            }else if (heading > 0.1 && heading < 90){
                lf.setPower(0.25 + (0.025 * heading));
                lb.setPower(0.25 + (0.025 * heading));
                rf.setPower(0.25 - (0.025 * heading));
                rb.setPower(0.25 - (0.025 * heading));
            } else {
                lf.setPower(0.25);
                lb.setPower(0.25);
                rf.setPower(0.25);
                rb.setPower(0.25);
            }

            //Telemetry Info for Diagnostics
            telemetry.addLine()
                    .addData("Alpha Output", "%.3f", colors.alpha)
                    .addData("Heading Output", "%.3f", heading)
                    .addData("Loop Count", "%,3f", countRed);
            telemetry.update();

            //If Statement to Detect the Red Line and Break the Loop
            if (colors.alpha < 0.2) {
                stopRobot();
                foundRed = true;
            }
            countRed++;
        }
    }


    //Method to Shoot Rings for Designated Time (seconds)
    public void shootRing(int wantedTime) {
        long time = System.currentTimeMillis();
        long _time = time + (wantedTime * 1000);

        while (System.currentTimeMillis() < _time) {
            rl1.setPower(-0.8460);
            rl2.setPower(-0.8460);

            _rf.setPosition(0.4);
            sleep(300);
            _rf.setPosition(0.8);
            sleep(300);

            rs.setPower(1);
        }
    }


    public void turn180() {
        while (opModeIsActive() && !turned180) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

            rf.setPower(-0.35);
            lf.setPower(0.35);
            rb.setPower(-0.35);
            lb.setPower(0.35);

            telemetry.addData("heading", heading);
            telemetry.update();

            if (heading <= 181 && heading >= 179) {
                stopRobot();
                turned180 = true;
            }
        }
    }



    public void strafe(String direction, int speed, double length) {
        double calcPosition = length * (100* 280/(16.9646003294*4 *8.8 * 1.0555555556));
        int setPosition = (int) Math.round(calcPosition);

        if (direction.equals("r")) {
            lf.setTargetPosition(setPosition);
            rf.setTargetPosition(-setPosition);
            lb.setTargetPosition(-setPosition);
            rb.setTargetPosition(setPosition);
        } else if (direction.equals("l")) {
            lf.setTargetPosition(-setPosition);
            rf.setTargetPosition(setPosition);
            lb.setTargetPosition(setPosition);
            rb.setTargetPosition(-setPosition);
        } else {
            stopRobot();
        }

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
}
