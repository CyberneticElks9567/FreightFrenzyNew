package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Post2020", group = "TeleOp")
@Disabled
public class Post2020 extends LinearOpMode
{
    OpMode opmode;
    float launchPower;
    int spin;

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

            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad1.a)
            {
                spin++;
            }
            if (spin==1)
            {
                h.servoSpin.setPower(1);
                h.servoSpin2.setPower(1);
            }
            if (spin==2)
            {
                h.servoSpin.setPower(0);
                h.servoSpin2.setPower(0);
            }
            if (spin==3)
            {
                h.servoSpin.setPower(.5);
                h.servoSpin2.setPower(.5);
                spin = 0;
            }
            /*if (gamepad1.right_trigger == 1)
            {
                h.motorLaunch.setPower(1);
                h.motorLaunch2.setPower(1);
            }
            if (gamepad1.left_trigger == 1)
            {
                h.motorLaunch.setPower(0);
                h.motorLaunch2.setPower(0);
            }*/

        }
    }
}
