package org.firstinspires.ftc.teamcode;
import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class turnTest extends LinearOpMode {
    public void runOpMode()
    {
        RobotController rc = new RobotController(hardwareMap, telemetry, "blue");
        waitForStart();

        DcMotorEx leftDeadwheel = hardwareMap.get(DcMotorEx.class, "leftSlide"); //left tick/rev = -2002.6 (forward)
        DcMotorEx rightDeadwheel = hardwareMap.get(DcMotorEx.class, "rightSlide"); //right tick/rev = -2004.4 (forward)
        DcMotorEx perpDeadwheel = hardwareMap.get(DcMotorEx.class, "perpendicularOdo");

        leftDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        final double deadwheelRadius = 1.6; //cm
        final double ticksPerRev = -2003.5;
        final double deadwheelDistance = 35.25; //cm; distance between deadwheels //13.875in
        final double angle0 = leftDeadwheel.getCurrentPosition() - rightDeadwheel.getCurrentPosition() / deadwheelDistance;
        double currentAngle; //degrees
        double cmLeft;
        double cmRight;
        double cmPerp;
        final double perpDistance = 17.78; //cm

        //field 365.75cm x 365.75cm
        final double x0 = 0;
        final double y0 = 0;
        double deltaCenterX;
        double deltaCenterY;
        double currentX; //cm
        double currentY; //cm

        while(!isStopRequested())
        {

            cmLeft = (double)leftDeadwheel.getCurrentPosition() / ticksPerRev  * 2 * Math.PI * deadwheelRadius;
            cmRight = (double)rightDeadwheel.getCurrentPosition() / ticksPerRev * 2 * Math.PI * deadwheelRadius;
            cmPerp = (double)perpDeadwheel.getCurrentPosition() / ticksPerRev * 2 * Math.PI * deadwheelRadius;
            currentAngle = angle0 + ((cmLeft - cmRight) / deadwheelDistance);
            currentAngle %= 2 * Math.PI;

            deltaCenterX = ((cmLeft + cmRight) / 2);
            deltaCenterY = (cmPerp - (perpDistance * currentAngle));

            currentX = x0 + (deltaCenterX * Math.cos(Math.toRadians(currentAngle))) - (deltaCenterY * Math.sin(Math.toRadians(currentAngle)));
            currentY = y0 + (deltaCenterX * Math.sin(Math.toRadians(currentAngle))) + (deltaCenterY * Math.cos(Math.toRadians(currentAngle)));

            //telemetry.addData("\noriginal angle", angle0);
            telemetry.addData("delta angle", currentAngle);

            telemetry.addData("\nx", currentX);
            telemetry.addData("y", currentY);
            telemetry.addData("deltaCenterX", deltaCenterX);
            telemetry.addData("deltaCenterY", deltaCenterY);
            telemetry.addData("currentAngle", Math.toDegrees(currentAngle));

            telemetry.update();
        }
    }
}