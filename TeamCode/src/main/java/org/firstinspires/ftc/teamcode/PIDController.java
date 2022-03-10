package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    double Kp;
    double Ki;
    double Kd;

    double derivative;
    double error;
    double integralSum = 0;
    double lastError = 0;
    double out;

    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double update(double target, double state) {
        error = target - state;

        ElapsedTime timer = new ElapsedTime();

        error = target - state;

        derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error + timer.seconds());

        out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;
        timer.reset();

        return out;

    }
}
