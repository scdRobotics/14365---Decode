
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
    public static double deadwheelDistance = 35; //120.955555; //34.798; //cm; distance between deadwheels //13.875in
    public static double perpDistance = -10.4775; //8.382; //cm //17.78 x 8.382cm (3.3in) //19.656711932568987006907830941549cm
    public static double parallelOffset = 5.08;

    //red human player zone (22.5, 22.5, 0)
    //blue human player zone(343.26,22.5, 0)

    static final public Pose blueHumanPlayer = new Pose(343.26, 22.5, 0);
    static final public Pose redHumanPlayer = new Pose(22.5, 22.5, 0);
    static final public Point redGoalPosition = new Pose(365.76, 365.76);
    static final public Point blueGoalPosition = new Pose(0, 365.76);
    static final public Pose topBlueStart = new Pose(142.5, 22.5, 0);
    static final public Pose topRedStart = new Pose(223.26, 22.5, 0);
    static final public Pose bottomBlueStart = new Pose(38.9247, 308.3108, -0.75705983410735577799);
    static final public Pose bottomRedStart = new Pose(308.1273, 318.3225, 0.75705983410735577799);

    final static double ticksPerRev = -2000; //-2003.5
    final static double deadwheelRadius = 1.6; //cm

    final static double deadwheelCircumference = 10.048;

    String color;


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



        telemetry.addLine("Odometry Initilized!");

        this.motors = List.of(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

        this.leftDeadwheel = hardwareMap.get(DcMotorEx.class, "frontLeftMotor"); //left tick/rev = -2002.6 (forward)
        this.rightDeadwheel = hardwareMap.get(DcMotorEx.class, "frontRightMotor"); //right tick/rev = -2004.4 (forward)
        this.rightDeadwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        this.perpDeadwheel = hardwareMap.get(DcMotorEx.class, "perpendicularOdo");

        this.leftDeadwheel.setDirection(DcMotorSimple.Direction.REVERSE);

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
    static double currentX; //cm
    static double currentY; //cm

    double lastFrameAngle = 0;

    double lastCmLeft, lastCmRight, lastCmPerp;

    public void update() {

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
        currentY += dStrafe * sin + dForward * cos;
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

        telemetry.addData("deltaCenterX", deltaCenterX);
        telemetry.addData("deltaCenterY", deltaCenterY);
        telemetry.addData("currentAngle", Math.toDegrees(currentAngle));
        telemetry.addData("left deadwheel", leftDeadwheel.getCurrentPosition());
        telemetry.addData("right deadwheel", rightDeadwheel.getCurrentPosition());
        telemetry.addData("perpendicular deadwheel", perpDeadwheel.getCurrentPosition());
    }
    public void telemetry()
    {

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

    public double getInchesFromMotorTicks(double motorTicks)
    {
        return (motorTicks/2000*deadwheelCircumference)/2.54;
    }

    public double getCmFromMotorTicks(double motorTicks)
    {
        return getInchesFromMotorTicks(motorTicks)*2.54;
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
            double x = blueGoalPosition.x - getX(); //adjacent / height
            double y = blueGoalPosition.y - getY(); //opposite / base

            return Math.atan(x/y);
        }
        if(color.equals("red"))
        {
            double x = redGoalPosition.x - getX(); //adjacent / height
            double y = redGoalPosition.y - getY(); //opposite / base

            return Math.atan(x/y);
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