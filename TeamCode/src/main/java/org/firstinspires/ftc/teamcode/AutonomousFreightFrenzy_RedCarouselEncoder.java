package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Carousel Autonomous Encoder", group = "Linear OpMode")
public class AutonomousFreightFrenzy_RedCarouselEncoder extends LinearOpMode {
    /**
     * Programmer:    Sean Pakros
     * Date Created:  12/09/21 (bit earlier then this as it was made during the competition then pushed later)
     * Purpose: Old autonomous for if we start by the red carousel, this one was encoder based so it was a bit
     * more accurate then the time one.
     */
    OpMode opmode;

    private String Date;
    private ElapsedTime runtime = new ElapsedTime();

    public int currentDegrees;

    long start = System.currentTimeMillis();

    int driveTime;


    @Override
    public void runOpMode()
    {
        Hardware h = new Hardware();

        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        h.imu = hardwareMap.get(BNO055IMU.class, "imu");
        h.imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !h.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", h.imu.getCalibrationStatus().toString());
        telemetry.update();
        h.servoIntake.setPosition(1);

        /*h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        h.motorFrontRight.setTargetPosition(0);
        h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorBackRight.setTargetPosition(0);
        h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorFrontLeft.setTargetPosition(0);
        h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorBackLeft.setTargetPosition(0);
        h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
        /*h.MRgyro.calibrate();
        while(h.MRgyro.isCalibrating() && opModeIsActive())
        {
            telemetry.update();
            telemetry.addData("MRgyro:", "calibrating");
        }
        telemetry.addData("Calibration", "complete");
        telemetry.addData("Initialization ", "complete");
        telemetry.addData("Heading: ", h.MRgyro.getIntegratedZValue());
        telemetry.update();*/




        waitForStart();

            /*h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
            /*h.motorArm.setTargetPosition(0);
            h.motorArm.setPower(1);
            h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

            telemetry.addData("motorFrontLeft encoder value: ",h.motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight encoder value: ",h.motorFrontRight.getCurrentPosition());
            telemetry.addData("motorBackLeft encoder value: ",h.motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight encoder value: ",h.motorBackRight.getCurrentPosition());
            telemetry.update();

            h.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            h.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            h.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            /*h.drive(true,5,.3);

            h.strafe(false,23,.3);

            h.motorCarousel.setPower(.3);
            h.sleep(4000);
            h.motorCarousel.setPower(0);

            h.drive(true,5,.3);*/



            h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            h.motorFrontLeft.setTargetPosition(440);
            h.motorFrontRight.setTargetPosition(440);
            h.motorBackLeft.setTargetPosition(440);
            h.motorBackRight.setTargetPosition(440);

            h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            h.setDrivePower((float) 0.2);


            h.sleep(2000);


            h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            h.motorFrontLeft.setTargetPosition(-2100);
            h.motorFrontRight.setTargetPosition(2100);
            h.motorBackLeft.setTargetPosition(2100);
            h.motorBackRight.setTargetPosition(-2100);

            h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.setDrivePower((float) 0.2);

            h.sleep(3000);

            h.motorCarousel.setPower(.3);
            h.sleep(5000);

            h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            h.motorCarousel.setPower(0);

            h.motorFrontLeft.setTargetPosition(1270);
            h.motorFrontRight.setTargetPosition(1270);
            h.motorBackLeft.setTargetPosition(1270);
            h.motorBackRight.setTargetPosition(1270);

            h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            h.setDrivePower((float) 0.2);

            /** PARK HAS CONCLUDED **/


            ///////////////////////////////////////////////////////////

            /** DRIVE FORWARD TO PREPARE FOR CAROUSEL **/
           /* h.TestDrive(true,20,.5);
            h.sleep(270);

            /** STRAFE TO CAROUSEL **/
            /*h.strafe(true,10,.5);
            h.sleep(1350);

            /** SPIN CAROUSEL **/
           /* h.motorCarousel.setPower(.3);
            h.sleep(3000);

            /** DRIVE TO PARKING ZONE **/
           /* h.TestDrive(true, 10,.5);
            h.sleep(700);*/

            /*h.strafe(true,50,.7);


            h.sleep(2500);
            h.turn(360,.5,.1);*/
            /*h.drive(true,28,1);
            h.sleep(1000);
            h.motorArm.setTargetPosition(190);
            h.sleep(1000);
            h.drive(true,5,.5);
            h.sleep(600);
            h.motorArm.setTargetPosition(0);
            h.sleep(1000);
            h.drive(false,35,.7);
            h.sleep(1000);
            h.motorArm.setTargetPosition(190);
            h.sleep(1000);
            h.strafe(true,70,1);*/
        /*
            h.motorArm.setTargetPosition(150);
            h.sleep(1000);
            h.servoStopper.setPosition(.65);
            h.sleep(1000);
            h.motorArm.setTargetPosition(0);
            h.sleep(1000);
            h.drive(true,35,.6);
        */
            /*h.drive(true, 15, .6);

            h.motorSwivel.setTargetPosition(200);
            h.motorSwivel.setPower(.8);

            h.servoSwivel.setPosition(1);

            h.motorArm.setTargetPosition(600);
            h.motorArm.setPower(.6);

            h.turn(90, .5, .3);

            h.drive(true, 60, .9);

            h.motorArm.setTargetPosition(100);

            h.servoSwivel.setPosition(100);

            h.turn(180, .5, .3);

            h.drive(true, 65, .9);

            h.turn(90, .5, .3);

            h.motorSwivel.setTargetPosition(500);

            h.servoSwivel.setPosition(1000);

            h.motorArm.setTargetPosition(600);

            h.turn(90, .5, .3);

            h.drive (true, 60, .9);

            h.motorArm.setTargetPosition(100);
            h.servoSwivel.setPosition(100);*/
        while (opModeIsActive())
        {
            telemetry.addData("motorFrontLeft encoder value: ",h.motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight encoder value: ",h.motorFrontRight.getCurrentPosition());
            telemetry.addData("motorBackLeft encoder value: ",h.motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight encoder value: ",h.motorBackRight.getCurrentPosition());
            telemetry.update();
        }
    }
}