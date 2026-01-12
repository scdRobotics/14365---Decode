package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Odometry {
    Telemetry telemetry;
    DcMotorEx leftDeadwheel;
    DcMotorEx rightDeadwheel;
    DcMotorEx perpDeadwheel;

    // CHANGE YEAR TO YEAR AND KILL THE BUILD TEAM IF THE CENTER OF THE ROBOT IS NOT THE CENTER OF ROTATION.
    final static double deadwheelDistance = 34.798; //cm; distance between deadwheels //13.875in
    final static double perpDistance = -17.78; //cm //17.78 x 8.382cm (3.3in) //19.656711932568987006907830941549cm


    final static double ticksPerRev = -2000; //-2003.5
    final static double deadwheelRadius = 1.6; //cm

    String color;

    private static final Point blueGoalPosition = new Point(365.76, 0);
    private static final Point redGoalPosition = new Point(365.76, 365.76);


    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    List<DcMotor> motors;

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry, String color, boolean isAuto)
    {
        this.telemetry = telemetry;

        this.frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        this.backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        this.frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        this.backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        this.motors = List.of(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        this.leftDeadwheel = hardwareMap.get(DcMotorEx.class, "leftSlide"); //left tick/rev = -2002.6 (forward)
        this.rightDeadwheel = hardwareMap.get(DcMotorEx.class, "rightSlide"); //right tick/rev = -2004.4 (forward)
        rightDeadwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        this.perpDeadwheel = hardwareMap.get(DcMotorEx.class, "perpendicularOdo");

        if(isAuto)
        {
            leftDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            perpDeadwheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            angle0 = Math.PI;
            x0 = 0;
            y0 = 0;
        }

        this.color = color;
    }
    public Odometry(HardwareMap hardwareMap, Telemetry telemetry, String color, boolean isAuto, Pose startPose)
    {
        this(hardwareMap, telemetry, color, isAuto);
        if(isAuto)
        {
            angle0 = startPose.angle;
            x0 = startPose.x;
            y0 = startPose.y;
        }
    }
    public Odometry(HardwareMap hardwareMap, Telemetry telemetry, String color)
    {
        this(hardwareMap, telemetry, color, false);
    }

    static double angle0; //leftDeadwheel.getCurrentPosition() - rightDeadwheel.getCurrentPosition() / deadwheelDistance
    static double currentAngle; //degrees

    static double cmLeft;
    static double cmRight;
    static double cmPerp;

    static double x0;
    static double y0;

    double deltaCenterX;
    double deltaCenterY;
    double currentX; //cm
    double currentY; //cm

    public void update()
    {
        cmLeft = (double)leftDeadwheel.getCurrentPosition() / ticksPerRev  * 2 * Math.PI * deadwheelRadius;
        cmRight = (double)rightDeadwheel.getCurrentPosition() / ticksPerRev * 2 * Math.PI * deadwheelRadius;
        cmPerp = (double)perpDeadwheel.getCurrentPosition() / ticksPerRev * 2 * Math.PI * deadwheelRadius;
        currentAngle = angle0 + ((cmLeft - cmRight) / deadwheelDistance);
        currentAngle %= 2 * Math.PI;

        deltaCenterX = ((cmLeft + cmRight) / 2);
        deltaCenterY = (cmPerp + (perpDistance * currentAngle));


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
    public void odometryTelemetry()
    {
        //telemetry.addData("\noriginal angle", angle0);
        telemetry.addData("delta angle", currentAngle);

        telemetry.addData("\nx", currentX);
        telemetry.addData("y", currentY);

        telemetry.addData("\nangle to goal", getAngleToGoal());
        telemetry.addData("distance to goal ", getDistToGoal());

        telemetry.addData("deltaCenterX", deltaCenterX);
        telemetry.addData("deltaCenterY", deltaCenterY);
        telemetry.addData("currentAngle", Math.toDegrees(currentAngle));

        telemetry.addData("left deadwheel", leftDeadwheel.getCurrentPosition());
        telemetry.addData("right deadwheel", rightDeadwheel.getCurrentPosition());
    }
    public double getAngle()
    {
        return currentAngle;
    }
    public Point getPosition()
    {
        return new Point(currentX, currentY);
    }
    public Pose getPose()
    {
        return new Pose(currentX, currentY, currentAngle);
    }
    public double getX()
    {
        return currentX;
    }
    public double getY()
    {
        return currentY;
    }

    public double getDistToGoal()
    {
        if(color.equals("blue"))
        {
            double a = Math.abs(getX() - blueGoalPosition.x);
            double b = Math.abs(getY() - blueGoalPosition.y);
            return Math.sqrt(Math.pow(a,2) + Math.pow(b,2));
        }
        else if(color.equals("red"))
        {
            double a = Math.abs(getX() - redGoalPosition.x);
            double b = Math.abs(getY() - redGoalPosition.y);
            return Math.sqrt(Math.pow(a,2) + Math.pow(b,2));
        }
        return -404;
    }
    public double getAngleToGoal()
    {
        if(color.equals("blue"))
        {
            double a = blueGoalPosition.x - getX(); //adjacent / height
            double b = blueGoalPosition.y - getY(); //opposite / base

            return Math.atan(b/a);
        }
        if(color.equals("red"))
        {
            double a = redGoalPosition.x - getX(); //adjacent / height
            double b = redGoalPosition.y - getY(); //opposite / base

            return Math.atan(b/a);
        }
        return -404;
    }
    public void turnToAngle(double angle, float power)
    {
        double distMult = 0.3;
        while(getAngle() < angle)
        {
            this.update();
            double dist = Math.abs(getAngle()-angle);
            double p = power * Math.min(1, dist*distMult);
            frontLeftMotor.setPower(p);
            backLeftMotor.setPower(p);
            frontRightMotor.setPower(-p);
            backRightMotor.setPower(-p);
        }

        while(getAngle() > angle)
        {
            this.update();
            double dist = Math.abs(getAngle()-angle);
            double p = power * Math.min(1, dist*distMult);
            frontLeftMotor.setPower(-p);
            backLeftMotor.setPower(-p);
            frontRightMotor.setPower(p);
            backRightMotor.setPower(p);
        }
    }
}