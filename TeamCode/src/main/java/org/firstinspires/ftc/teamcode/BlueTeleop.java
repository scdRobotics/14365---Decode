package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class BlueTeleop extends LinearOpMode
{
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    public static double middleOfScreenX = 960;

    //public static final double leftSideMult = .8235;

    public static double angle = 45;

    final float interval = 0.1f;

    public int slidePosition;

    /**
     * The variable to store our instance of the AprilTag processor.
     */

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException
    {
        getCurrentGameTagLibrary();
        Point center = new Point();
        ArrayList<Integer> colorList;
        AprilTagDetection aprilTagDetection = new AprilTagDetection(hardwareMap, telemetry, "blue");
        Intake intake = new Intake(hardwareMap, telemetry);
        intake.openBottomServo(false);
        intake.openTopServo(false);
        Lifting lift = new Lifting(hardwareMap, telemetry);

        float power = 1;
        boolean lastFrameDPad = false;


        waitForStart();
        // Declare our motors
        // Make sure your ID's match your configuration
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

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        aprilTagDetection.initAprilTag();

        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            center = aprilTagDetection.telemetryAprilTag();

            double y = gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.left_stick_x;

            double frontLeftPower = -(y + x + rx);
            double backLeftPower = -(y - x + rx);
            double frontRightPower = -(y - x - rx);
            double backRightPower = -(y + x - rx);

            boolean lastFrameLeft = false;
            boolean lastFrameRight = false;
            boolean lastFrameRightTrigger = false;
            boolean lastFrameX = false;
            boolean lastFrameY = false;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            if(center.x < 920) telemetry.addLine("move left");
            if(center.x > 1120) telemetry.addLine("move right");

            if(gamepad2.right_trigger != 0 && !lastFrameRightTrigger)
            {
                if(aprilTagDetection.getDistance() < 0) telemetry.addLine("Camera cannot find AprilTag! \nplease try moving back");
                else intake.shoot3(intake.getPolynomialPower((float)aprilTagDetection.getDistance()));
            }
            if(gamepad2.left_trigger != 0)
            {
                intake.shootMotor.setVelocity(1000);
            }
            if(gamepad2.left_bumper)
            {
                intake.shootMotor.setVelocity(0);
            }
            if(gamepad2.right_bumper)
            {
                intake.dropBall();
            }
            if(gamepad2.a && aprilTagDetection.getDistance() > 0)
            {
                aprilTagDetection.turnToCenterGoal(motors, .4f, 3);
            }
            else intake.shootMotor.setPower(0);
            if(gamepad1.a) intake.servoTop.setPosition(intake.topOpenPos);

            telemetry.addData("frontRightMotor", frontRightMotor.getCurrentPosition());
            telemetry.addData("frontLeftMotor", frontLeftMotor.getCurrentPosition());
            telemetry.addData("backRightMotor", backRightMotor.getCurrentPosition());
            telemetry.addData("backLeftMotor", backLeftMotor.getCurrentPosition());

            if(aprilTagDetection.getDistance() < 0)
            {
                telemetry.addLine("|||||||| Cannot find April Tag!!! ||||||||");
            }
            else telemetry.addData("aprilTag distance", aprilTagDetection.getDistance());

            telemetry.addData("aprilTag X", center.x);
            telemetry.addData("center of screen X", aprilTagDetection.middleOfScreenX);

            telemetry.addData("center.x > middleScreen-10", center.x + " > " + (middleOfScreenX-10));
            telemetry.addData("center.x < middleScreen+10", center.x + " < " + (middleOfScreenX+10));

            lastFrameRight = gamepad1.dpad_right;
            lastFrameLeft = gamepad1.dpad_left;
            lastFrameRightTrigger = gamepad1.right_trigger != 0;
            lastFrameDPad = gamepad1.dpad_down || gamepad1.dpad_up;
            lastFrameX = gamepad1.x;
            lastFrameY = gamepad1.y;

            telemetry.update();
        }
    }
}