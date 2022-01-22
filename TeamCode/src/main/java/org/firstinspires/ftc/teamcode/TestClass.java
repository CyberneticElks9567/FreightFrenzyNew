package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "test class", group = "TeleOp")
@Disabled
public class TestClass extends LinearOpMode
{
    OpMode opmode;

    @Override
    public void runOpMode() {
        Hardware h = new Hardware();


        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }

        telemetry.addData("Main Initialization ", "complete");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {



            telemetry.addData("motorFrontLeft: ", h.motorFrontLeft.getPower());
            telemetry.addData("motorFrontRight: ", h.motorFrontRight.getPower());
            telemetry.addData("motorBackLeft: ", h.motorBackLeft.getPower());
            telemetry.addData("motorBackRight: ", h.motorBackRight.getPower());
            telemetry.addData("left trigger: ", gamepad1.left_trigger);
            telemetry.update();
            //h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            h.motorFrontLeft.setPower(gamepad1.left_stick_y);
            h.motorFrontRight.setPower(gamepad1.left_stick_y);
            h.motorBackLeft.setPower(gamepad1.left_stick_y);
            h.motorBackRight.setPower(gamepad1.left_stick_y);

        }
    }
}
