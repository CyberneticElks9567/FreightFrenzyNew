package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Blue Freight side Detector", group="Auto")
public class FreightOpenCVBlueFreight extends LinearOpMode {
    Hardware h = new Hardware();
    OpenCvCamera webCam;
    int ShippingElementPosFromLeft;


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FreightDetector detector = new FreightDetector(telemetry);
        webCam.setPipeline(detector);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Error", "Camera not able to open");
            }
        });
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

        waitForStart();
        telemetry.addData("iteration:", 3);
        telemetry.update();
        switch (detector.getLocation()) {
            case LEFT: //Top
                /** GETTING READY TO DROP BLOCK **/
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(1550);
                h.motorFrontRight.setTargetPosition(-1550);
                h.motorBackLeft.setTargetPosition(-1550);
                h.motorBackRight.setTargetPosition(1550);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.setDrivePower((float) 0.2);
                h.sleep(5000);

                h.drivePureEncoder(true,800,.3);
                /** DROPPING BLOCK **/

                h.motorWinch.setTargetPosition(400);
                h.motorWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorWinch.setPower(.5);

                while (h.motorWinch.isBusy() && !isStopRequested())
                {
                    telemetry.addData("motorWinch Pos: ", h.motorWinch.getCurrentPosition());
                    telemetry.update();
                }

                h.servoIntake.setPosition(0);

                h.sleep(300);

                h.motorWinch.setTargetPosition(0);
                h.motorWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorWinch.setPower(.5);

                break;
            case MIDDLE://middle
                /** GETTING READY TO DROP BLOCK **/
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(1550);
                h.motorFrontRight.setTargetPosition(-1550);
                h.motorBackLeft.setTargetPosition(-1550);
                h.motorBackRight.setTargetPosition(1550);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.setDrivePower((float) 0.2);
                h.sleep(5000);

                h.drivePureEncoder(true,800,.3);
                /** DROPPING BLOCK **/
                h.motorArm.setTargetPosition(968);
                h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorArm.setPower(.5);

                while (h.motorArm.isBusy() && !isStopRequested())
                {
                    telemetry.addData("motorArm Pos: ", h.motorArm.getCurrentPosition());
                    telemetry.update();
                }

                h.motorWinch.setTargetPosition(140);
                h.motorWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorWinch.setPower(.5);

                while (h.motorWinch.isBusy() && !isStopRequested())
                {
                    telemetry.addData("motorWinch Pos: ", h.motorWinch.getCurrentPosition());
                    telemetry.update();
                }

                h.servoIntake.setPosition(0);

                h.sleep(300);

                h.motorWinch.setTargetPosition(0);
                h.motorWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorWinch.setPower(.5);
                break;
            case RIGHT://bottom
                /** GETTING READY TO DROP BLOCK **/
                h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                h.motorFrontLeft.setTargetPosition(1550);
                h.motorFrontRight.setTargetPosition(-1550);
                h.motorBackLeft.setTargetPosition(-1550);
                h.motorBackRight.setTargetPosition(1550);

                h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.setDrivePower((float) 0.2);
                h.sleep(5000);

                h.drivePureEncoder(true,800,.3);
                /** DROPPING BLOCK **/
                h.motorArm.setTargetPosition(500);
                h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorArm.setPower(.5);

                while (h.motorArm.isBusy() && !isStopRequested())
                {
                    telemetry.addData("motorArm Pos: ", h.motorArm.getCurrentPosition());
                    telemetry.update();
                }

                h.motorWinch.setTargetPosition(250);
                h.motorWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorWinch.setPower(.5);

                while (h.motorWinch.isBusy() && !isStopRequested())
                {
                    telemetry.addData("motorWinch Pos: ", h.motorWinch.getCurrentPosition());
                    telemetry.update();
                }

                h.servoIntake.setPosition(0);

                h.sleep(300);

                h.motorWinch.setTargetPosition(0);
                h.motorWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorWinch.setPower(.5);
                break;
        }

        webCam.stopStreaming();

        h.turnIMU(90,.5,.3);
        h.drive(true,45,.7);
        h.motorArm.setTargetPosition(0);
        h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorArm.setPower(.5);


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
        /*h.drive(true,8,.5);
        h.turnIMU(-90,.5,.3);
        h.drive(true,32,.7);*/

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
