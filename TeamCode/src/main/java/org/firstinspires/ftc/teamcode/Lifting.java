package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class Lifting extends LinearOpMode {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    DcMotor leftSlide, rightSlide;

    public static final int slideStartPosition = 25;

    public Lifting(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        rightSlide = hardwareMap.dcMotor.get("rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        rightSlide = hardwareMap.dcMotor.get("rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive())
        {
            telemetry.addData("leftSlidePos", leftSlide.getCurrentPosition());
            telemetry.addData("rightSlidePos", rightSlide.getCurrentPosition());

            telemetry.update();
        }
    }

    public void telemetrySlideData()
    {
        telemetry.addData("leftSlide encoder", leftSlide.getCurrentPosition());
        telemetry.addData("rightSlide encoder", rightSlide.getCurrentPosition());
    }

    public void moveRightSlideToPosition(int position)
    {
        rightSlide.setTargetPosition(position);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setPower(.8);
    }
    public void moveLeftSlideToPosition(int position)
    {
        leftSlide.setTargetPosition(position);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(.8);
    }
    public void moveSlidesToPosition(int position)
    {
        moveRightSlideToPosition(position);
        moveLeftSlideToPosition(position);
    }

    public int getSlidesPos()
    {
        if(leftSlide.getCurrentPosition() != rightSlide.getCurrentPosition()) return Integer.MIN_VALUE;
        else return leftSlide.getCurrentPosition();
    }
}