package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCurrentGameTagLibrary;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

import java.util.ArrayList;

@TeleOp
public class Test extends LinearOpMode
{
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    public static double middleOfScreenX = 320;
    public static double middleOfScreenY = 240;

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
        AprilTagDetection aprilTagDetection = new AprilTagDetection(hardwareMap, telemetry);
        //Physics physics = new Physics(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        intake.openBottomServo(false);
        intake.openTopServo(false);
        Lifting lift = new Lifting(hardwareMap, telemetry);

        lift.moveSlidesToPosition(lift.slideStartPosition);
        slidePosition = lift.slideStartPosition;

        float power = 1;
        boolean lastFrameDPad = false;


        waitForStart();
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotor[] motors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};

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
            intake.update();
            center = aprilTagDetection.telemetryAprilTag();
            //if(aprilTagDetection.colors.size() != 0 && colorList.size() == 0);
            colorList = aprilTagDetection.colors;
            if(colorList.size() != 0) intake.init(colorList);
            telemetry.addData("Color List", colorList);
            telemetry.addData("Center", center);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

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


            if(center.x > middleOfScreenX - 5) telemetry.addLine("move right");
            if(center.x < middleOfScreenX + 5) telemetry.addLine("move left");

            if(center.y > middleOfScreenY - 5) telemetry.addLine("move down");
            if(center.y < middleOfScreenY + 5) telemetry.addLine("move up");

            //hello from runako muvirmi
            if(gamepad1.dpad_up && !lastFrameDPad) power+=interval;
            if(gamepad1.dpad_down && !lastFrameDPad) power-=interval;
            if(power < 0) power = 0;
            if(power > 1) power = 1;

            if(gamepad1.dpad_right && !lastFrameRight) intake.rotateCounterClockwise();
            if(gamepad1.dpad_left && !lastFrameLeft) intake.rotateClockwise();
            if(gamepad1.right_trigger != 0 && !lastFrameRightTrigger)
            {
                //intake.orderedShoot(physics.findGoodVel(angle, aprilTagDetection.getDistance()));
                intake.shoot3(1);
            }
            //else intake.shoot(0);
            if(gamepad1.left_trigger != 0) intake.shootMotor.setPower(-1);
            else intake.shootMotor.setPower(0);
            if(gamepad1.a) intake.servoTop.setPosition(intake.topOpenPos);

            /*if(gamepad1.x && !lastFrameX)
            {
                slidePosition -= 1;
                lift.moveSlidesToPosition(slidePosition);
            }
            if(gamepad1.y && !lastFrameY)
            {
                slidePosition += 1;
                lift.moveSlidesToPosition(slidePosition);
            }
            if(gamepad1.b) lift.moveSlidesToPosition(slidePosition);*/


            telemetry.addData("frontRightMotor", frontRightMotor.getCurrentPosition());
            telemetry.addData("frontLeftMotor", frontLeftMotor.getCurrentPosition());
            telemetry.addData("backRightMotor", backRightMotor.getCurrentPosition());
            telemetry.addData("backLeftMotor", backLeftMotor.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.addData("calculated Distance", aprilTagDetection.getDistance());
            telemetry.addData("slide position", slidePosition);
            lift.telemetrySlideData();

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