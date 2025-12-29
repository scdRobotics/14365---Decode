package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;


@TeleOp
public class TPICalculator extends LinearOpMode {

    final static double POWER = 1;
    public static final int slideStartPosition = 25;

    @Override
    public void runOpMode()
    {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotor shootMotor = hardwareMap.dcMotor.get("shootMotor");

        List<DcMotor> motors = List.of(frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor);

        Lifting lifting = new Lifting(hardwareMap, telemetry);

        lifting.moveSlidesToPosition(lifting.slideStartPosition);


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        waitForStart();
        if (isStopRequested()) return;

        long lastFrameMillis = System.currentTimeMillis();
        long millisRan;
        long ticksPerMilli;
        long ticksPerRev = 100; //measure
        long revPerMin;

        while(opModeIsActive())
        {
            telemetry.addData("rightSlide target position", lifting.rightSlide.getTargetPosition());
            telemetry.addData("rightSlide current position", lifting.rightSlide.getCurrentPosition());

            double frontLeftMotorTicks = frontLeftMotor.getCurrentPosition();
            double backLeftMotorTicks = backLeftMotor.getCurrentPosition();
            double frontRightMotorTicks = frontRightMotor.getCurrentPosition();
            double backRightMotorTicks = backRightMotor.getCurrentPosition();

            double avgTicks = (frontLeftMotorTicks+backLeftMotorTicks+frontRightMotorTicks+backRightMotorTicks)/4;

            if(gamepad1.a)
            {
                for (DcMotor motor : motors) motor.setPower(POWER);
                telemetry.addLine("Moving...\n");
            }
            else
            {
                for (DcMotor motor : motors) motor.setPower(0);
                telemetry.addLine("Press 'A' to move forwards \n");
            }

            if(gamepad1.x)
            {
                shootMotor.setPower(1);
                telemetry.addLine("shooting");
            }
            else telemetry.addLine("press 'X' to shoot");

            millisRan = System.currentTimeMillis() - lastFrameMillis;
            lastFrameMillis = System.currentTimeMillis();
            ticksPerMilli = (long)shootMotor.getCurrentPosition() / millisRan;
            revPerMin = ticksPerMilli / ticksPerRev * 60000;
            telemetry.addData("shoot motor rpm", revPerMin);

            telemetry.addData("frontLeftMotor ticks", frontLeftMotorTicks);
            telemetry.addData("backLeftMotor ticks", backLeftMotorTicks);
            telemetry.addData("frontRightMotor ticks", frontRightMotorTicks);
            telemetry.addData("backRightMotor ticks", backRightMotorTicks + "\n");

            telemetry.addData("shootMotor ticks", shootMotor.getCurrentPosition());

            telemetry.addData("averageTicks, divide by inches traveled/Degrees rotated to get TPI/TPR", avgTicks);

            telemetry.update();
        }

    }

}
