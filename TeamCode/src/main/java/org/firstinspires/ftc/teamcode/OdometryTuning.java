package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@TeleOp
public class OdometryTuning extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        Odometry odometry = new Odometry(hardwareMap, telemetry, "blue", true);

        Lifting lift = new Lifting(hardwareMap, telemetry);
        lift.keepSlideUp();
        final double deadwheelDistanceInterval = 0.025;
        final double perpDistanceInterval = 0.01;
        final double parallelDistanceInterval = 0.01;

        waitForStart();
        odometry.leftDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometry.rightDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odometry.perpDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        List<DcMotor> motors = List.of(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while(!isStopRequested())
        {
            if(gamepad1.rightBumperWasPressed()) odometry.setPose(new Pose(0,0,0));
            odometry.update();
            if(gamepad1.dpadDownWasPressed()) odometry.deadwheelDistance -= deadwheelDistanceInterval;
            if(gamepad1.dpadUpWasPressed())odometry.deadwheelDistance += deadwheelDistanceInterval;

            if(gamepad1.yWasPressed()) odometry.perpDistance -= perpDistanceInterval;
            if(gamepad1.aWasPressed()) odometry.perpDistance += perpDistanceInterval;

            if(gamepad1.xWasPressed()) odometry.parallelOffset -= parallelDistanceInterval;
            if(gamepad1.bWasPressed())odometry.parallelOffset += parallelDistanceInterval;

            double y = gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.left_stick_x;

            double frontLeftPower = -(y + x - rx);
            double backLeftPower = -(y - x - rx);
            double frontRightPower = (y - x + rx);
            double backRightPower = -(y + x + rx);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("Trackwidth", odometry.deadwheelDistance);
            telemetry.addData("perp Offset", odometry.perpDistance);
            telemetry.addData("parallel Offset", odometry.parallelOffset);

            telemetry.addData("pose", odometry.getPose());
            telemetry.addData("\nleft deadwheel pos", odometry.leftDeadwheel.getCurrentPosition());
            telemetry.addData("right deadwheel pos", odometry.rightDeadwheel.getCurrentPosition());
            telemetry.addData("perp deadwheel pos", odometry.perpDeadwheel.getCurrentPosition());

            telemetry.addData("\nfrontRightWheel pos", frontRightMotor.getCurrentPosition());
            telemetry.addData("frontLeftWheel pos", frontLeftMotor.getCurrentPosition());
            telemetry.addData("backRightWheel pos", backRightMotor.getCurrentPosition());
            telemetry.addData("backLeftWheel pos", backLeftMotor.getCurrentPosition());



            telemetry.update();
        }
    }
}
