package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

public class SensorTester extends LinearOpMode {

    //set Motors
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    //set Gyro
    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro mrGyro;

    @Override
    public void runOpMode() throws InterruptedException {

        //hardware Mapping Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        //reversing Motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //run Without Encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //hardware Mapping Gyro
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;

        //position Variables
        int currentHeading;
        int targetHeading = 0;

        //sleep, calibrate, wait for start
        sleep(1000);
        mrGyro.calibrate();
        waitForStart();
        //calibrate Gyro
        while (mrGyro.isCalibrating()){
        }
        //opMode
        while (opModeIsActive()){
        //methods go here
        }
    }

    public void turnTo(int targetHeading) throws InterruptedException{


        while (mrGyro.isCalibrating()) {
            int currentHeading = mrGyro.getIntegratedZValue();
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

                currentHeading = mrGyro.getIntegratedZValue();

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
}