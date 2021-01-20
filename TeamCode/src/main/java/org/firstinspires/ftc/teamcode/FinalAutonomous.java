package org.firstinspires.ftc.teamcode;

//import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;


@Autonomous(name = "FinalAutonomous", group = "comp")


/*******************************************************************************
 *                              COMP PROGRAM: START                             *
 *******************************************************************************/

// INITILIZATION
public class FinalAutonomous extends LinearOpMode {

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
    NormalizedRGBA colors;
    boolean foundRed = false;
    boolean foundWhite = false;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double heading;
    boolean turned180 = false;
    boolean didStrafe = false;
    boolean isStraight = false;

    boolean isShooting = false;


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
        ts.setPosition(0.99);
        bs.setPosition(0);
        _rf.setPosition(0.8);
        sleep(200);

        as.setPower(1);
        sleep(2000);
        forceStop();


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

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        //Waiting for start via Player
        waitForStart();

        //Beginning Loop for Program
        if (opModeIsActive()) {

            encoders("n");

            rl1.setPower(-0.8460);
            rl2.setPower(-0.8460);
            sleep(1000);

            long time = System.currentTimeMillis();
            long _time = time + 7500;

            while (System.currentTimeMillis() < _time) {
                rl1.setPower(-0.8460);
                rl2.setPower(-0.8460);

                _rf.setPosition(0.4);
                sleep(300);
                _rf.setPosition(0.8);
                sleep(300);

                rs.setPower(1);
            }
            forceStop();
            sleep(250);

            if(quadStack) {
                encoders("n");
                senseLine("white", 0.25);
                moveInches(48);
                encoders("n");
                turn(0.3, 94);
                moveInches(-33);
                encoders("n");
                dropWobble();
                moveInches(38);
                encoders("n");
                turn(0.4, 180);
                senseLine("white", 0.3);
                forceStop();
            } else if(singleStack) {
                encoders("n");
                senseLine("white", 0.25);
                sleep(1000);
                moveInches(24);
                turn(0.5, 100);
                moveInches(-12);
                dropWobble();
                moveInches (12);
                turn(0.5, 180);
                senseLine("white", 0.25);
                forceStop();
            } else {
                encoders("n");
                senseLine("white", 0.25); // 0.25
                turn(0.25, 90);
                moveInches(-28);
                dropWobble();
                moveInches(4);
            }
            as.setPower(-1);
            sleep(2000);
            forceStop();
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
       // int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
        //        "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       // TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       // tfodParameters.minResultConfidence = 0.8f;
        //tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
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

    public void encoders(String answer) {
        if (answer.equals("y")) {
            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (answer.equals("n")) {
            lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    //Method to Move Robot @ Designated Speed & Duration
    public void moveRobot(double rfSpeed, double lfSpeed, double rbSpeed, double lbSpeed, long dur) {
        rf.setPower(rfSpeed);
        lf.setPower(lfSpeed);
        rb.setPower(rbSpeed);
        lb.setPower(lbSpeed);
        sleep(dur);
    }



    public void dropWobble() {
        bs.setPosition(0.9);
        sleep(100);
        ts.setPosition(0.5);
        sleep(25);
    }


    //Method to Find & Move to the White Line
    void senseLine(String color, double speed) {
        //Needed (non-changing) Variables
        final float[] hsvValues = new float[3];
        foundRed = false;
        foundWhite = false;
        int countRed = 0;
        int countWhite = 0;

        //If the "foundRed" Boolean is False, Run Loop
        while ((!foundRed && !foundWhite) && opModeIsActive()) {
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
                    .addData("Loop Count", countRed);
            telemetry.update();

            if (color.equals("red")) {
                //If Statement to Detect the Red Line and Break the Loop
                if (colors.alpha < 0.2) {
                    stopRobot();
                    foundRed = true;
                }
                countRed++;
            } else if (color.equals("white")) {
                if (colors.alpha > 0.5) {
                    stopRobot();
                    foundWhite = true;
                }
                countWhite++;
            } else {
                stopRobot();
            }
        }
    }


    //Method to Shoot Rings for Designated Time (seconds)
    public void shootRing(int wantedTime, double speed) {
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


    void turn(double speed, int angleMeasure) {
        turned180 = false;
        while (opModeIsActive() && !turned180) {
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



    public void strafe(String direction, int ticks, double length) {
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

        lf.setVelocity(ticks);
        rf.setVelocity(ticks);
        lb.setVelocity(ticks);
        rb.setVelocity(ticks);

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


    void moveInches(double lengthUsingInches) {

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void straightenRobot(double currentHeading, double speed) {
        if (currentHeading > 2) {
            while (opModeIsActive() && !isStraight) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

                rf.setPower(-speed);
                lf.setPower(speed);
                rb.setPower(-speed);
                lb.setPower(speed);

                telemetry.addData("Heading Output", "%.3f", heading);
                telemetry.update();

                if (heading <= 1 && heading >= -1) {
                    stopRobot();
                    isStraight = true;
                }
            }
        } else if (currentHeading < -2) {
            while (opModeIsActive() && !isStraight) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                double heading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

                rf.setPower(speed);
                lf.setPower(-speed);
                rb.setPower(speed);
                lb.setPower(-speed);

                telemetry.addData("Heading Output", "%.3f", heading);
                telemetry.update();

                if (heading <= 1 && heading >= -1) {
                    stopRobot();
                    isStraight = true;
                }
            }
        } else {
            stopRobot();
        }
        isStraight = false;
    }
}