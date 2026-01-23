package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class CookieClicker extends LinearOpMode {
    long cookies = 0;
    long cookieInterval = 1;
    long cookieIntervalInterval = 1;
    long cookieIntervalIntervalInterval = 1;

    long cookieIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalIntervalInterval = Long.MAX_VALUE;
    long grandmas = 0;
    long grandmaInterval = 1;
    long buyInterval = 1;
    long startTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (!isStopRequested())
        {
            if (gamepad2.aWasPressed()) cookies += cookieInterval;
            if (gamepad2.bWasPressed() && cookies >= 25 * buyInterval) {
                cookies -= 25 * buyInterval;
                cookieInterval += cookieIntervalInterval * buyInterval;
            }
            if (gamepad2.xWasPressed() && cookieInterval > 25 * buyInterval) {
                cookieInterval -= 25 * buyInterval;
                cookieIntervalInterval += cookieIntervalIntervalInterval * buyInterval;
            }
            if(gamepad2.yWasPressed() && cookieIntervalInterval > 25 * buyInterval)
            {
                cookieIntervalInterval -= 25 * buyInterval;
                cookieIntervalIntervalInterval += 1 * buyInterval;
            }

            if(gamepad2.dpadUpWasPressed() && cookies >= 500 * buyInterval)
            {
                cookies -= 500 * buyInterval;
                grandmas += grandmaInterval * buyInterval;
            }
            if(gamepad2.dpadDownWasPressed() && grandmas >= 50 * buyInterval)
            {
                grandmas -= 50 * buyInterval;
                grandmaInterval += 1 * buyInterval;
            }
            grandmaTimer();

            if(gamepad2.dpadLeftWasPressed()) buyInterval /= 10;
            if(gamepad2.dpadRightWasPressed()) buyInterval *= 10;
            buyInterval = Math.max(buyInterval, 1);

            telemetry.addData("a: cookies",  cookies);
            telemetry.addData("b: cookie interval", cookieInterval);
            telemetry.addData("x: cookie interval interval", cookieIntervalInterval);
            telemetry.addData("y: cookie interval interval interval", cookieIntervalIntervalInterval);
            telemetry.addData("d up: grandmas", grandmas);
            telemetry.addData("d down: grandma interval", grandmaInterval);
            telemetry.addData("d left/right: buy interval", buyInterval);
            telemetry.update();
        }
    }
    public void grandmaTimer()
    {
        if(startTime == 0) startTime = System.currentTimeMillis();
        if(System.currentTimeMillis() - startTime >= 1000)
        {
            cookies += grandmas;
            startTime = System.currentTimeMillis();
        }
    }
}
