package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Testing", group = "TeleOp")
public class TeleOp2022Testing extends LinearOpMode
{
    OpMode opmode;
    public enum ArmState {
        ARM_START,
        ARM_EXTEND,
        ARM_DUMP,
        ARM_RETRACT
    };
    ArmState armState = ArmState.ARM_START;
    @Override
    public void runOpMode() {
        Hardware h = new Hardware();


        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }
        h.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        h.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        h.motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        h.motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
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
        telemetry.addData("Main Initialization ", "complete");
        telemetry.update();

        boolean pressedLastIterationIntake = false;
        int armLevel = 3;
        boolean bButton = false, aButtonPressed = false, bButtonPressed = false;
        h.servoIntake.setPosition(1);
        ElapsedTime armTimer = new ElapsedTime();
        waitForStart();
        while (opModeIsActive())
        {
            boolean pressedIntake = gamepad1.x;
            //telemetry.addData("range", String.format("%.01f in", h.distanceSensor.getDistance(DistanceUnit.INCH)));
            //telemetry.addData("Distance: ",h.distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("motorWinch Position: ", h.motorWinch.getCurrentPosition() + " busy =" + h.motorWinch.isBusy());
            telemetry.addData("motorArm Position: ", h.motorArm.getCurrentPosition() + " busy =" + h.motorArm.isBusy());
            telemetry.addData("servoIntake: ", h.servoIntake.getPosition());
            telemetry.addData("motorFrontLeft encoder value: ",h.motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight encoder value: ",h.motorFrontRight.getCurrentPosition());
            telemetry.addData("motorBackLeft encoder value: ",h.motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight encoder value: ",h.motorBackRight.getCurrentPosition());
            telemetry.addData("Degrees: ", h.getIntegratedHeading());
            telemetry.addData("State: ", armState);
            telemetry.update();
            boolean slow = false;
            //h.driveFieldRelative(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            h.driveOmniDir(gamepad1.left_stick_x / 2, gamepad1.left_stick_y / 2, gamepad1.right_stick_x / 2);
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


            /*if(pressedIntake & !pressedLastIterationIntake)
            {
                if(h.servoIntake.getPosition() > .55)
                {
                    h.servoIntake.setPosition(.4); //0 .1
                }
                else
                {
                    h.servoIntake.setPosition(1); //1 .3
                }

            }
            if (gamepad1.y)
            {
                h.servoIntake.setPosition(0);
            }*/
            if(gamepad1.start)
            {
                h.servoIntake.setPosition(.57);
            }
            if(gamepad1.back)
            {
                h.servoIntake.setPosition(1);
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
                h.motorArm.setTargetPosition(0);
                h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorArm.setPower(.8);
            }
            if (gamepad1.right_bumper /*&& h.motorArm.getPosition() > low limit*/)
            {
                h.motorArm.setTargetPosition(500);
                h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                h.motorArm.setPower(1);
            }
            switch(armState)
            {
                case ARM_START:
                // Waiting for some input
                    telemetry.addData("In: ", "ARM_START");
                    if (gamepad1.x)
                    {
                        // x is pressed, start extending
                        h.motorWinch.setTargetPosition(400);
                        h.motorWinch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        h.motorWinch.setPower(1);
                        armState = armState.ARM_EXTEND;
                    }
                break;
                case ARM_EXTEND:
                    // check if the left has finished extending,
                    // otherwise do nothing.
                    telemetry.addData("In: ", "ARM_EXTEND");
                    if (Math.abs(h.motorWinch.getCurrentPosition() - 400) < 10)
                    {
                        // our threshold is within
                        // 10 encoder ticks of our target.
                        // this is pretty arbitrary, and would have to be
                        // tweaked for each robot.

                        // set the lift dump to dump
                        h.servoIntake.setPosition(0);

                        armTimer.reset();
                        armState = armState.ARM_DUMP;
                    }
                    break;
                case ARM_DUMP:
                    if (armTimer.seconds() >= .5)
                    {
                        // The robot waited long enough to drop the material, time to start
                        // retracting the lift
                        h.motorWinch.setTargetPosition(0);

                        armState = armState.ARM_RETRACT;
                    }
                    break;
                case ARM_RETRACT:
                    if (Math.abs(h.motorArm.getCurrentPosition() - 0) < 10)
                    {
                        armState = armState.ARM_START;
                    }
                    break;
                default:
                    // should never be reached, as armState should never be null
                    armState = armState.ARM_START;
            }

            /*switch (armLevel)
            {
                case 0:
                    h.motorArm.setTargetPosition(1470);
                    h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    h.motorArm.setPower(1);
                break;

                case 1:
                    h.motorArm.setTargetPosition(1148);
                    h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    h.motorArm.setPower(1);
                break;

                case 2:
                    h.motorArm.setTargetPosition(500);
                    h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    h.motorArm.setPower(1);
                break;

                case 3:
                    h.motorArm.setTargetPosition(0);
                    h.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    h.motorArm.setPower(.8);
                break;
            }
            if (gamepad1.a)
                if(!aButtonPressed)
                {
                    if (armLevel <= 3) {
                        armLevel += 1;
                    }
                    aButtonPressed = true;
                }
                else {}
            else
                aButtonPressed = false;
            if (gamepad1.b)
                if(!bButtonPressed)
                {
                    if(armLevel >= 0) {
                        armLevel -= 1;
                    }
                    bButtonPressed = true;
                }
                else {}
            else
                bButtonPressed = false;
*/
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
            /* if(!gamepad1.left_bumper && gamepad1.left_trigger == 0)
            {
                h.motorWinch.setPower(0);
            } */
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


            //pressedLastIterationIntake = pressedIntake;
        }
    }
}

