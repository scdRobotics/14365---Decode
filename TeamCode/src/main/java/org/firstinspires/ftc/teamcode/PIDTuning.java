package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

//@TeleOp
public class PIDTuning /*extends LinearOpMode*/ {
    /*static PIDFCoefficients pidCoefficients;
    static double p,i,d,f;

    public void runOpMode()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        DcMotorEx shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        pidCoefficients = new PIDFCoefficients(18,.8,1,0);

        p = pidCoefficients.p;
        i = pidCoefficients.i;
        d = pidCoefficients.d;
        f = pidCoefficients.f;

        final double TPR = 28;
        final double targetVel = 1700;

        int dir = 1;

        waitForStart();

        double startTime= 0;
        final double pauseTime = 10000;

        double TPS = 0;
        double lastTicks = shootMotor.getCurrentPosition();
        long lastFrameTime = 0;

        double interval = 0.01;

        while(!isStopRequested())
        {
            if(gamepad1.dpadUpWasPressed()) p += interval;
            if(gamepad1.dpadDownWasPressed()) p -= interval;

            if(gamepad1.dpadLeftWasPressed()) i -= interval;
            if(gamepad1.dpadRightWasPressed()) i += interval;

            if(gamepad1.yWasPressed()) d += interval;
            if(gamepad1.aWasPressed()) d -= interval;

            if(gamepad1.bWasPressed()) f += interval;
            if(gamepad1.xWasPressed()) f -= interval;

            if(gamepad1.leftBumperWasPressed()) interval -= 0.0025;
            if(gamepad1.rightBumperWasPressed()) interval += 0.0025;

            pidCoefficients = new PIDFCoefficients(p,i,d,f);
            shootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);

            shootMotor.setVelocity(targetVel*dir);
            if(Math.abs(shootMotor.getVelocity()) >= targetVel)
            {
                if(startTime == 0) startTime = System.currentTimeMillis();
                if(System.currentTimeMillis() - startTime >= pauseTime)
                {
                    dir = -dir;
                    startTime = 0;
                }
            }
            TPS = (shootMotor.getCurrentPosition() - lastTicks) / (System.currentTimeMillis() - lastFrameTime) / 1000;
            lastFrameTime = System.currentTimeMillis();
            lastTicks = shootMotor.getCurrentPosition();
            double RPM = TPS*60.0/TPR;

            telemetry.addData("RPM", RPM);
            telemetry.addData("velocity", shootMotor.getVelocity());
            telemetry.addData("TargetVel", targetVel*dir);
            telemetry.addData("p, i, d, f", p + ", " + i + "," + d + ", " + f);
            telemetry.addData("PID coefficients", pidCoefficients);
            telemetry.addData("interval", interval);
            telemetry.update();
        }
    }*/
}