package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Carousel Autonomous---------this one if by red carousel-------", group = "Linear OpMode")
public class AutonomousFreightFrenzy_RedCarousel extends LinearOpMode {

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
        h.servoIntake.setPosition(1);
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

            /*telemetry.addData("motorFrontLeft encoder value: ",h.motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight encoder value: ",h.motorFrontRight.getCurrentPosition());
            telemetry.addData("motorBackLeft encoder value: ",h.motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight encoder value: ",h.motorBackRight.getCurrentPosition());*/
            telemetry.update();

            /** MOVE FORWARD FOR 270 TO PREPARE FOR MOVE TO CAROUSEL **/
            h.motorFrontRight.setPower(-.5);
            h.motorBackRight.setPower(-.5);
            h.motorFrontLeft.setPower(.5);
            h.motorBackLeft.setPower(.5);
            h.sleep(270);
            h.setDrivePower(0);

            /** STRAFE LEFT FOR 1350 TO CAROUSEL **/
            h.motorFrontRight.setPower(-.5);
            h.motorBackRight.setPower(+.5);
            h.motorFrontLeft.setPower(-.5);
            h.motorBackLeft.setPower(+.5);
            h.sleep(1350);
            h.setDrivePower(0);

            /** SPIN CAROUSEL + **/
            h.motorCarousel.setPower(.3);
            h.sleep(3000);

            /** MOVE FORWARD 700 INTO PARKING ZONE**/
            h.motorFrontRight.setPower(-.5);
            h.motorBackRight.setPower(-.5);
            h.motorFrontLeft.setPower(.5);
            h.motorBackLeft.setPower(.5);
            h.sleep(700);
            h.setDrivePower(0);

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
        /*while (opModeIsActive())
        {
            telemetry.addData("motorFrontLeft encoder value: ",h.motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight encoder value: ",h.motorFrontRight.getCurrentPosition());
            telemetry.addData("motorBackLeft encoder value: ",h.motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight encoder value: ",h.motorBackRight.getCurrentPosition());
            telemetry.update();
        }*/
    }
}