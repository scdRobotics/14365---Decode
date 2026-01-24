package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class RobotController {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    Intake intake;
    AprilTagDetection aprilTagDetection;
    Odometry odometry;
    Lifting lift;

    IMU imu;
    double angleOffset;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    List<DcMotor> motors;
    List<DcMotor> motorEncoders;

    public RobotController(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, "both");
    }

    public RobotController(HardwareMap hardwareMap, Telemetry telemetry, String color) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        this.backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        this.frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        this.backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        this.intake = new Intake(hardwareMap, telemetry);
        this.aprilTagDetection = new AprilTagDetection(hardwareMap, telemetry, color);
        this.odometry = new Odometry(hardwareMap, telemetry, color, true);
        this.aprilTagDetection.initAprilTag();
        this.lift = new Lifting(hardwareMap, telemetry);
        lift.keepSlideUp();

        this.intake.servoBottom.setPosition(intake.bottomClosePos);
        this.intake.servoTop.setPosition(intake.topClosePos);

        this.imu = hardwareMap.get(IMU.class, "imu");

        angleOffset = imu.getRobotYawPitchRollAngles().getYaw();

        this.motors = List.of(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        this.motorEncoders = List.of(frontLeftMotor, frontRightMotor);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean motorsAreBusy() {
        for (DcMotor motor : motorEncoders) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }

    public void moveForward(float time, float power) {
        motors.get(0).setPower(-power);
        motors.get(1).setPower(-power);
        motors.get(2).setPower(power);
        motors.get(3).setPower(-power);
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime >= time) {
            odometry.update();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public void moveBackward(float time, float power) {
        motors.get(0).setPower(power);
        motors.get(1).setPower(power);
        motors.get(2).setPower(-power);
        motors.get(3).setPower(power);
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime <= time) {
            odometry.update();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public void moveLeft(float time, float power) {
        motors.get(0).setPower(power);
        motors.get(1).setPower(-power);
        motors.get(2).setPower(power);
        motors.get(3).setPower(-power);
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime <= time) {
            odometry.update();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public void moveRight(float time, float power) {
        motors.get(0).setPower(-power);
        motors.get(1).setPower(power);
        motors.get(2).setPower(-power);
        motors.get(3).setPower(power);
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime <= time) {
            odometry.update();
        }
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    /*public void moveForward(int inches, float power)
    {
        int ticks = (int)(inches*32.26);
        for (DcMotor motor : motors)
        {
            motor.setPower(power);
        }
        for (DcMotor motor : motorEncoders)
        {
            motor.setTargetPosition(ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        while(motorsAreBusy())
        {
            odometry.update();
        };
        for (DcMotor motor : motors)
        {
            motor.setPower(0);
        }
        for(DcMotor motor : motorEncoders)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void moveBackward(int inches, float power)
    {
        int ticks = (int)(inches*32.26);
        for (DcMotor motor : motors)
        {
            motor.setPower(power);
        }
        for (DcMotor motor : motorEncoders)
        {
            motor.setTargetPosition(-ticks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        while(motorsAreBusy())
        {
            odometry.update();
        };
        for (DcMotor motor : motors)
        {
            motor.setPower(0);
        }
        for(DcMotor motor : motorEncoders)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void moveLeft(int inches, float power)
    {
        int ticks = (int)(inches*32.26);
        motors.get(0).setTargetPosition(-ticks);
        motors.get(0).setPower(power);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //motors.get(1).setTargetPosition(ticks);
        motors.get(1).setPower(power);
        //motors.get(1).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motors.get(2).setTargetPosition(ticks);
        motors.get(2).setPower(power);
        motors.get(2).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //motors.get(3).setTargetPosition(-ticks);
        motors.get(3).setPower(power);
        //motors.get(3).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(motorsAreBusy())
        {
            odometry.update();
        };
        for (DcMotor motor : motors)
        {
            motor.setPower(0);
        }
        for(DcMotor motor : motorEncoders)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void moveRight(int inches, float power)
    {
        int ticks = (int)(inches*32.26);
        motors.get(0).setTargetPosition(ticks);
        motors.get(0).setPower(power);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //motors.get(1).setTargetPosition(-ticks);
        motors.get(1).setPower(power);
        //motors.get(1).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motors.get(2).setTargetPosition(-ticks);
        motors.get(2).setPower(power);
        motors.get(2).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //motors.get(3).setTargetPosition(ticks);
        motors.get(3).setPower(power);
        //motors.get(3).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(motorsAreBusy())
        {
            odometry.update();
        };
        for (DcMotor motor : motors)
        {
            motor.setPower(0);
        }
        for(DcMotor motor : motorEncoders)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
     */
    public void turnToAngle(float angle, float power, int precision) {
        for (int i = 1; i <= precision; i++) {

            while (imu.getRobotYawPitchRollAngles().getYaw() < angle) {
                motors.get(0).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(0).setPower(-power / i);

                motors.get(1).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(1).setPower(-power / i);

                motors.get(2).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(2).setPower(power / i);

                motors.get(3).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(3).setPower(power / i);

                odometry.update();
            }

            while (imu.getRobotYawPitchRollAngles().getYaw() > angle) {
                motors.get(0).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(0).setPower(power / i);

                motors.get(1).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(1).setPower(power / i);

                motors.get(2).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(2).setPower(-power / i);

                motors.get(3).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(3).setPower(-power / i);

                odometry.update();
            }

            for (DcMotor motor : motors) {
                motor.setPower(0);
            }
            for (DcMotor motor : motorEncoders) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        for (DcMotor motor : motorEncoders) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void shoot3(float power) {
        intake.shoot3Auto(power);
    }

    public void shoot(float power) {
        intake.shoot(power);
    }

    public void turnToCenterGoal(float power, int precision, int offset) {
        aprilTagDetection.turnToCenterGoal(motors, power, precision, offset);
    }

    public void turnToCenterGoal(float power, int precision) {
        aprilTagDetection.turnToCenterGoal(motors, power, precision);
    }

    public void moveToGoalDistance(double distance, float power, int precision) {
        aprilTagDetection.moveToGoalDistance(motors, distance, power, precision);
    }

    public float getPow() {
        while (aprilTagDetection.getDistance() < 0) {
            aprilTagDetection.telemetryAprilTag();
            telemetry.addData("distance", aprilTagDetection.getDistance());
            telemetry.update();
        }

        return intake.getPolynomialPower((float) aprilTagDetection.getDistance());
    }

    public float getPowLinear() {
        return intake.getPowLinear((float) aprilTagDetection.getDistance());
    }

    public void runCam() {
        aprilTagDetection.telemetryAprilTag();
    }

    public double getDist() {
        return aprilTagDetection.getDistance();
    }

    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw() - angleOffset;
    }

    public void turnToAngle(double angle, float power) {
        double distMult = 1;
        telemetry.addData("turn to angle", angle);
        telemetry.update();

        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (odometry.getAngle() > angle) {
            while (odometry.getAngle() > angle) {
                odometry.update();
                double dist = Math.abs(odometry.getAngle() - angle);
                double p = power * Math.min(1, dist * distMult);
                p = Math.max(0.3, p);
                frontLeftMotor.setPower(p);
                backLeftMotor.setPower(p);
                frontRightMotor.setPower(-p);
                backRightMotor.setPower(-p);
                telemetry.addData("power to turn at", p);
                telemetry.addData("currentAngle", odometry.getAngle());
                telemetry.addData("wanted angle", angle);
                telemetry.update();
            }
        } else {
            while (odometry.getAngle() < angle) {
                odometry.update();
                double dist = Math.abs(odometry.getAngle() - angle);
                double p = power * Math.min(1, dist * distMult);
                p = Math.max(0.3, p);
                frontLeftMotor.setPower(-p);
                backLeftMotor.setPower(-p);
                frontRightMotor.setPower(p);
                backRightMotor.setPower(p);
                telemetry.addData("power to turn at", p);
                telemetry.addData("currentAngle", odometry.getAngle());
                telemetry.addData("wanted angle", angle);
                telemetry.update();
            }
        }
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    public double getBearing() {
        return aprilTagDetection.getBearing();
    }
}
