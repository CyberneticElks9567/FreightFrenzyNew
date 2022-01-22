package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "2021 TeleOp", group = "TeleOp")
@Disabled
public class TeleOp2021 extends LinearOpMode
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
        boolean pressedLastIteration = false;
        int launchValue = 0;


        telemetry.addData("Main Initialization ", "complete");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {

            /*telemetry.addData("motorWinch Position: ", h.motorWinch.getCurrentPosition());
            telemetry.addData("motorSwivel Position: ", h.motorSwivel.getCurrentPosition());
            telemetry.addData("motorArm Position: ", h.motorArm.getCurrentPosition());*/

            //telemetry.addData("targetSwivel Value:", swivelTarget);
            //telemetry.addData("Distance: ",h.distanceSensor.getDistance(DistanceUnit.INCH));
            /*telemetry.addData("Alpha", h.colorSensor.alpha());
            telemetry.addData("Red  ", h.colorSensor.red());
            telemetry.addData("Green", h.colorSensor.green());
            telemetry.addData("Blue ", h.colorSensor.blue());*/

            telemetry.addData("motorFrontLeft: ", h.motorFrontLeft.getPower());
            telemetry.addData("motorFrontRight: ", h.motorFrontRight.getPower());
            telemetry.addData("motorBackLeft: ", h.motorBackLeft.getPower());
            telemetry.addData("motorBackRight: ", h.motorBackRight.getPower());
            telemetry.addData("left trigger: ", gamepad1.left_trigger);
            telemetry.addData("motorLaunch: ", h.motorLaunch.getPower());
            telemetry.addData("servoIntake: ",  h.servoIntake.getPosition());
            //telemetry.addData("launchValue: ",launchValue);
            //telemetry.addData("intakeValue", intakeValue);
            //telemetry.addData("range", String.format("%.01f in", h.distanceSensor.getDistance(DistanceUnit.INCH)));
            //telemetry.addData("Distance: ",h.distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            boolean pressed = gamepad1.y;
            if(gamepad1.dpad_up)
            {
                h.motorLaunch.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if(gamepad1.dpad_down)
            {
                h.motorLaunch.setDirection(DcMotorSimple.Direction.FORWARD);
            }



           /* if(gamepad1.y)
            {
                h.motorLaunch.setPower(gamepad1.left_trigger);
            }
            if(gamepad1.x)
            {
                h.motorLaunch.setPower(0);
            }*/
            /*if(gamepad1.left_trigger == 1 ) // Trigger Launcher Control
            {
                h.motorLaunch.setPower(1);
                //++h.launchValue;
            }
            else if (gamepad1.left_trigger < .5)
            {
                h.motorLaunch.setPower(0);
            }*/
            if (gamepad1.y)
            {
                h.motorLaunch.setPower(1);
            }
            else
            {
                h.motorLaunch.setPower(0);
            }
            if(pressed & !pressedLastIteration)
            {
                if(h.motorLaunch.getPower() == 0)
                {
                    h.motorLaunch.setPower(1);
                }
                else
                {
                    h.motorLaunch.setPower(0);
                }

            }
            pressedLastIteration = pressed;


            /*if (gamepad1.y) //y button toggle Launcher Control
            {
                ++launchValue;
            }
            switch (launchValue)
            {
                case 1:
                    h.motorLaunch.setPower(1);
                    break;
                case 2:
                    h.motorLaunch.setPower(0);
                    launchValue = 0;
                    break;
            }*/
            if(gamepad1.a)
            {
                h.servoIntake.setPosition(.7);
            }
            if(gamepad1.b)
            {
                h.servoIntake.setPosition(1);
            }
            /*if (gamepad1.a) //Intake Control
            {
                ++intakeValue;
            }
            switch (intakeValue)
            {
                case 1:
                    h.motorIntake.setPower(1);
                    h.motorLaunch.setPower(-0.3);
                    break;
                case 2:
                    h.motorIntake.setPower(0);
                    h.motorLaunch.setPower(0);
                    intakeValue = 0;
                    break;
                default:
                    intakeValue = 0;
            }*/

            /*if (gamepad1.left_bumper) //High Position
            {
                h.motorArm.setTargetPosition(1);
                h.motorArm.setPower(.6);

            }
            if (gamepad1.right_bumper) //Low Position
            {
                h.motorArm.setTargetPosition(-1);
                h.motorArm.setPower(.6);
            }
            if (!h.motorArm.isBusy())
            {
                h.motorArm.setPower(0);
            }*/



        }
    }
}
