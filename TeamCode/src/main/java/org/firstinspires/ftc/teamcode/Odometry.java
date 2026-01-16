package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Odometry {
    Telemetry telemetry;
    DcMotorEx leftDeadwheel;
    DcMotorEx rightDeadwheel;
    DcMotorEx perpDeadwheel;

    DcMotor frontLeftMotor, frontRightMotor, backRightMotor, backLeftMotor;
    List<DcMotor> motors;


    // CHANGE YEAR TO YEAR
    final static double deadwheelDistance = 35.25; //cm; distance between deadwheels //13.875in
    final static double perpDistance = 17.78; //cm
    final static double parallelOffset = 9.5; //help :(


    final static double ticksPerRev = -2003.5;
    final static double deadwheelRadius = 1.6; //cm

    String color;

    public static final Point blueGoalPosition = new Pose(365.75, 0);
    public static final Point redGoalPosition = new Pose(365.75, 365.75);
    public static final Pose redHumanPlayer = new Pose(22.5, 22.5);
    public static final Pose blueHumanPlayer = new Pose(343.25, 22.5);

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry, String color, boolean isAuto)
    {
        this.telemetry = telemetry;

        this.frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        this.backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        this.frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        this.backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        telemetry.addLine("Odometry Initilized!");

        this.motors = List.of(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        this.leftDeadwheel = hardwareMap.get(DcMotorEx.class, "leftSlide"); //left tick/rev = -2002.6 (forward)
        this.rightDeadwheel = hardwareMap.get(DcMotorEx.class, "rightSlide"); //right tick/rev = -2004.4 (forward)
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

    double lastCmRight;
    double lastCmLeft;
    double lastCmPerp;

    public void update()
    {
        //field 365.75cm x 365.75cm
        cmLeft = (double)leftDeadwheel.getCurrentPosition() / ticksPerRev  * 2 * Math.PI * deadwheelRadius;
        cmRight = (double)rightDeadwheel.getCurrentPosition() / ticksPerRev * 2 * Math.PI * deadwheelRadius;
        cmPerp = (double)perpDeadwheel.getCurrentPosition() / ticksPerRev * 2 * Math.PI * deadwheelRadius;

        currentAngle = angle0 + ((cmLeft - cmRight) / deadwheelDistance);
        currentAngle %= 2 * Math.PI;
        currentAngle -= Math.PI;

        deltaCenterX = ((cmLeft + cmRight) / 2);
        deltaCenterY = (cmPerp - (perpDistance * currentAngle));

        currentX = x0 + (deltaCenterX * Math.cos(Math.toRadians(currentAngle))) - (deltaCenterY * Math.sin(Math.toRadians(currentAngle)));
        currentY = y0 + (deltaCenterX * Math.sin(Math.toRadians(currentAngle))) + (deltaCenterY * Math.cos(Math.toRadians(currentAngle)));

        // --- Read encoders (absolute) ---
        double leftPos = leftDeadwheel.getCurrentPosition();
        double rightPos = rightDeadwheel.getCurrentPosition();
        double perpPos = perpDeadwheel.getCurrentPosition();

        // --- Convert ticks to distance ---
        double cmLeft = leftPos / ticksPerRev * 2 * Math.PI * deadwheelRadius;
        double cmRight = rightPos / ticksPerRev * 2 * Math.PI * deadwheelRadius;
        double cmPerp = perpPos / ticksPerRev * 2 * Math.PI * deadwheelRadius;

        // --- Delta distances ---
        double dL = cmLeft - lastCmLeft;
        double dR = cmRight - lastCmRight;
        double dC = cmPerp - lastCmPerp;

        lastCmLeft = cmLeft;
        lastCmRight = cmRight;
        lastCmPerp = cmPerp;

        // --- Delta heading ---
        double dTheta = (dR - dL) / deadwheelDistance;

        // --- Robot-relative motion (CORRECTED) ---
        double dForward = ((dL + dR) / 2.0) - (parallelOffset * dTheta);
        double dStrafe  = dC - (perpDistance * dTheta);

        // --- Rotate into field frame ---
        double midHeading = currentAngle + dTheta / 2.0;

        double cos = Math.cos(midHeading);
        double sin = Math.sin(midHeading);

        currentX += dStrafe * cos - dForward * sin;
        currentY -= dStrafe * sin + dForward * cos;
        currentAngle += dTheta;


        boolean isNeg;
        if(currentAngle < 0)
        {
            isNeg = true;
            currentAngle -= Math.PI;
        }
        else {
            isNeg = false;
            currentAngle += Math.PI;
        }
        currentAngle %= 2*Math.PI;
        if(isNeg) currentAngle += Math.PI;
        else currentAngle -= Math.PI;

        // Normalize heading
        //currentAngle = (currentAngle + Math.PI) % (2 * Math.PI) - Math.PI;
    }
    public void odometryTelemetry()
    {
        //telemetry.addData("\noriginal angle", angle0);
        telemetry.addData("delta angle", currentAngle);

        telemetry.addData("\nx", currentX);
        telemetry.addData("y", currentY);

        telemetry.addData("\nangle to goal", getAngleToGoal());
        telemetry.addData("distance to goal ", getDistToGoal());

        /*telemetry.addData("deltaCenterX", deltaCenterX);
        telemetry.addData("deltaCenterY", deltaCenterY);
        telemetry.addData("currentAngle", Math.toDegrees(currentAngle));*/
    }
    public double getAngle(AngleUnit unit)
    {
        if(unit == AngleUnit.DEGREES) return Math.toDegrees(currentAngle);
        if(unit == AngleUnit.RADIANS) return currentAngle;
        else return -404;
    }
    public double getAngle()
    {
        return getAngle(AngleUnit.DEGREES);
    }
    public Point getPosition()
    {
        return new Point(currentX, currentY);
    }
    public Pose getPose()
    {
        return new Pose(getX(), getY(), getAngle());
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

    public void setPose(Pose pose)
    {
        currentX = pose.x;
        currentY = pose.y;
        currentAngle = pose.angle;
    }
    public double getPower()
    {
        //y=-0.00303763x^{3}+3.41269x^{2}-1274.7416x+160066.513
        double x = this.getDistToGoal();
        if(x < 300) return 0.000047338*Math.pow(x, 3) - 0.0246386*Math.pow(x,2) + 4.50147*x + 1110.05994;//y=0.000047338x^{3}-0.0246386x^{2}+4.50147x+1110.05994
        if(x < 358) return 1725;
        if(x > 395) return 1800;
        return (-0.00303763*Math.pow(x,3)) + (3.41269*Math.pow(x,2)) - (1274.7416*x) + (160066.513);
    }

    public void turnToAngle(double angle, float power)
    {
        double distMult = 1;
        telemetry.addData("turn to angle", angle);
        telemetry.update();

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(getAngle() > angle)
        {
            while(getAngle() > angle)
            {
                this.update();
                double dist = Math.abs(getAngle()-angle);
                double p = power * Math.min(1, dist*distMult);
                p = Math.max(0.3,p);
                this.frontLeftMotor.setPower(p);
                this.backLeftMotor.setPower(p);
                this.frontRightMotor.setPower(-p);
                this.backRightMotor.setPower(-p);
                telemetry.addData("power to turn at", p);
                telemetry.update();
            }
        }
        else
        {
            while(getAngle() < angle)
            {
                this.update();
                double dist = Math.abs(getAngle()-angle);
                double p = power * Math.min(1, dist*distMult);
                p = Math.max(0.3,p);
                this.frontLeftMotor.setPower(-p);
                this.backLeftMotor.setPower(-p);
                this.frontRightMotor.setPower(p);
                this.backRightMotor.setPower(p);
                telemetry.addData("power to turn at", p);
                telemetry.update();
            }
        }

        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }
}