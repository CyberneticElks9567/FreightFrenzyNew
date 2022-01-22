package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "2022 TeleOp -CHOOSE THIS ONE-", group = "TeleOp")
public class TeleOp2022 extends LinearOpMode
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

        boolean pressedLastIterationIntake = false;
        boolean pressedLastIterationCarouselReverse = false;
        boolean slow = false;
        double armSpeedUp = -1;
        double armSpeedDown = .8;

        waitForStart();
        while (opModeIsActive())
        {
            boolean pressedIntake = gamepad1.x;
            //telemetry.addData("range", String.format("%.01f in", h.distanceSensor.getDistance(DistanceUnit.INCH)));
            //telemetry.addData("Distance: ",h.distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("motorWinch Position: ", h.motorWinch.getCurrentPosition() + " busy =" + h.motorWinch.isBusy());
            telemetry.addData("motorArm Position: ", h.motorArm.getCurrentPosition() + " busy =" + h.motorArm.isBusy());
            telemetry.addData("servoIntake: ", h.servoIntake.getPosition());
            telemetry.addData("slowdown bool: ", slow);
            telemetry.addData("armSpeedUp: ", armSpeedUp);
            telemetry.addData("armSpeedDown: ", armSpeedDown);
            telemetry.addData("motorFrontLeft encoder value: ",h.motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight encoder value: ",h.motorFrontRight.getCurrentPosition());
            telemetry.addData("motorBackLeft encoder value: ",h.motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight encoder value: ",h.motorBackRight.getCurrentPosition());
            telemetry.update();
            slow = gamepad1.a;
            h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            if(gamepad1.dpad_left || gamepad2.dpad_left)
            {
                h.motorFrontLeft.setPower(-.2);
                h.motorFrontRight.setPower(.2);
                h.motorBackLeft.setPower(-.2);
                h.motorBackRight.setPower(.2);
            }
            else if (gamepad1.dpad_right || gamepad2.dpad_right)
            {
                h.motorFrontLeft.setPower(.2);
                h.motorFrontRight.setPower(-.2);
                h.motorBackLeft.setPower(.2);
                h.motorBackRight.setPower(-.2);
            }
            if(gamepad1.dpad_up || gamepad2.dpad_up)
            {
                h.motorFrontLeft.setPower(.2);
                h.motorFrontRight.setPower(.2);
                h.motorBackLeft.setPower(.2);
                h.motorBackRight.setPower(.2);
            }
            else if(gamepad1.dpad_down || gamepad2.dpad_down)
            {
                h.motorFrontLeft.setPower(-.2);
                h.motorFrontRight.setPower(-.2);
                h.motorBackLeft.setPower(-.2);
                h.motorBackRight.setPower(-.2);
            }


            if(pressedIntake & !pressedLastIterationIntake)
            {
                if(h.servoIntake.getPosition() >= .3)
                {
                    h.servoIntake.setPosition(.1); //0 .1
                }
                else
                {
                    h.servoIntake.setPosition(1); //1 .3
                }

            }

            if (gamepad1.y)
            {
                h.servoIntake.setPosition(0);
            }

            //.1 is open

            /*if(pressedSl & !pressedLastIterationCarousel)
            {
                if(h.motorCarousel.getPower() == 0 && gamepad2.b)
                {
                    h.motorCarousel.setPower(.3); //.3
                }
                else if(h.motorCarousel.getPower() == 0)
                {
                    h.motorCarousel.setPower(-.3); //-.3
                }
                else
                {
                    h.motorCarousel.setPower(0);
                }

            }*/
            if (gamepad2.b)
            {
                h.motorCarousel.setPower(.4);
            }
            if (gamepad2.x)
            {
                h.motorCarousel.setPower(-.4);
            }
            if (!gamepad2.x && !gamepad2.b)
            {
                h.motorCarousel.setPower(0);
            }

            /*if(pressedCarouselReverse & !pressedLastIterationCarouselReverse)
            {
                if(h.motorCarousel.getPower() == 0)
                {
                    h.motorCarousel.setPower(-.3);
                }
                else
                {
                    h.motorCarousel.setPower(0);
                }

            }*/
            if(gamepad1.right_trigger > .01 /*&& h.motorArm.getPosition() < high limit*/)
            {
                h.motorArm.setPower(armSpeedDown);
            }
            if (gamepad1.right_bumper /*&& h.motorArm.getPosition() > low limit*/)
            {
                h.motorArm.setPower(armSpeedUp);
            }
            if(!gamepad1.right_bumper && gamepad1.right_trigger == 0)
            {
                h.motorArm.setPower(0);
            }

            if(slow)
            {
                armSpeedDown = .4;
                armSpeedUp = -.5;
            }
            else
            {
                armSpeedDown = .8;
                armSpeedUp = -1;
            }
            
            /*if(gamepad1.b)
            {
                h.motorArm.setTargetPosition(500);
                h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorArm.setPower(.3);
            }
            if(gamepad1.a)
            {
                h.motorArm.setTargetPosition(0);
                h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorArm.setPower(.3);
            }*/


            if(gamepad1.left_trigger > .01 && h.motorWinch.getCurrentPosition() < 450)
            {
                h.motorWinch.setPower(.5);
            }
            if (gamepad1.left_bumper && h.motorWinch.getCurrentPosition() >= -10)
            {
                h.motorWinch.setPower(-.5);
            }
            if(!gamepad1.left_bumper && gamepad1.left_trigger == 0)
            {
                h.motorWinch.setPower(0);
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
            /*if (gamepad1.y)
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
            pressedLastIteration = pressed;*/


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
            /*if(gamepad1.a)
            {
                h.servoIntake.setPosition(.7);
            }
            if(gamepad1.b)
            {
                h.servoIntake.setPosition(1);
            }*/
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


            pressedLastIterationIntake = pressedIntake;
        }
    }
}
