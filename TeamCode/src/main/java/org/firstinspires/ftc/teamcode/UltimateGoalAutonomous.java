package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled @Autonomous(name = "UltimateGoal Autonomous", group = "Linear OpMode")

public class UltimateGoalAutonomous extends LinearOpMode {

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

        h.MRgyro.calibrate();
        while(h.MRgyro.isCalibrating() && opModeIsActive())
        {
            telemetry.update();
            telemetry.addData("MRgyro:", "calibrating");
        }
        telemetry.addData("Calibration", "complete");
        telemetry.addData("Initialization ", "complete");
        telemetry.addData("Heading: ", h.MRgyro.getIntegratedZValue());
        telemetry.update();




        waitForStart();

            h.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            h.motorArm.setTargetPosition(0);
            h.motorArm.setPower(1);
            h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("motorFrontLeft encoder value: ",h.motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight encoder value: ",h.motorFrontRight.getCurrentPosition());
            telemetry.addData("motorBackLeft encoder value: ",h.motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight encoder value: ",h.motorBackRight.getCurrentPosition());
            telemetry.update();


    }
}