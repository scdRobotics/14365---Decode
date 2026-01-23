package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class CheckHardwareMap extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        waitForStart();
        while(!isStopRequested()) {
            if (gamepad1.a) backLeftMotor.setPower(1);
            else backLeftMotor.setPower(0);

            if (gamepad1.b) backRightMotor.setPower(1);
            else backRightMotor.setPower(0);

            if (gamepad1.x) frontLeftMotor.setPower(1);
            else frontLeftMotor.setPower(0);

            if (gamepad1.y) frontRightMotor.setPower(1);
            else frontRightMotor.setPower(0);

            telemetry.addLine("y: frontRight");
            telemetry.addLine("x: frontLeft");
            telemetry.addLine("b: backRight");
            telemetry.addLine("a: backLeft");
            telemetry.update();
        }
    }
}
