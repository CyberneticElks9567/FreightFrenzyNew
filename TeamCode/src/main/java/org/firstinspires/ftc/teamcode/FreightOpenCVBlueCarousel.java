package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Blue Carousel Detector-----DON't RUN", group="Auto")
public class FreightOpenCVBlueCarousel extends LinearOpMode {
    Hardware h = new Hardware();
    OpenCvCamera webCam;
    public enum Position {
        TOP,
        MIDDLE,
        BOTTOM
    }
    Position position;

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
        switch (detector.getLocation()) {
            case LEFT: //middle
                position = Position.MIDDLE;
                break;
            case MIDDLE://bottom
                position = Position.BOTTOM;
                break;
            case RIGHT://top
                position = Position.TOP;
        }
        webCam.stopStreaming();

        telemetry.addData("motorFrontLeft encoder value: ",h.motorFrontLeft.getCurrentPosition());
        telemetry.addData("motorFrontRight encoder value: ",h.motorFrontRight.getCurrentPosition());
        telemetry.addData("motorBackLeft encoder value: ",h.motorBackLeft.getCurrentPosition());
        telemetry.addData("motorBackRight encoder value: ",h.motorBackRight.getCurrentPosition());
        telemetry.update();

        h.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        h.turnIMU(90,.5,.3);

        h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        h.motorFrontLeft.setTargetPosition(-1600);
        h.motorFrontRight.setTargetPosition(-1600);
        h.motorBackLeft.setTargetPosition(-1600);
        h.motorBackRight.setTargetPosition(-1600);

        h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.setDrivePower((float) 0.2);

        h.sleep(3000);

        h.motorCarousel.setPower(-.3);
        h.sleep(5000);
        h.motorCarousel.setPower(0);

        h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        h.motorFrontLeft.setTargetPosition(1750);
        h.motorFrontRight.setTargetPosition(-1750);
        h.motorBackLeft.setTargetPosition(-1750);
        h.motorBackRight.setTargetPosition(1750);

        h.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.setDrivePower((float) 0.2);

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
