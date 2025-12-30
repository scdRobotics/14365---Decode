package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    IMU imu;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    List<DcMotor> motors;
    public RobotController(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        this.backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        this.frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        this.backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        this.intake = new Intake(hardwareMap, telemetry);
        this.aprilTagDetection = new AprilTagDetection(hardwareMap, telemetry);

        this.imu = hardwareMap.get(IMU.class, "imu");

        this.motors = List.of(frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        this.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void moveForward(int ticks, float power)
    {
        for (DcMotor motor : motors)
        {
            motor.setTargetPosition(ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void moveBackward(int ticks, float power)
    {
        for (DcMotor motor : motors)
        {
            motor.setTargetPosition(-ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void moveLeft(int ticks, float power)
    {
        motors.get(0).setTargetPosition(-ticks);
        motors.get(0).setPower(power);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motors.get(1).setTargetPosition(ticks);
        motors.get(1).setPower(power);
        motors.get(1).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motors.get(2).setTargetPosition(ticks);
        motors.get(2).setPower(power);
        motors.get(2).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motors.get(3).setTargetPosition(-ticks);
        motors.get(3).setPower(power);
        motors.get(3).setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveRight(int ticks, float power)
    {
        motors.get(0).setTargetPosition(ticks);
        motors.get(0).setPower(power);
        motors.get(0).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motors.get(1).setTargetPosition(-ticks);
        motors.get(1).setPower(power);
        motors.get(1).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motors.get(2).setTargetPosition(-ticks);
        motors.get(2).setPower(power);
        motors.get(2).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motors.get(3).setTargetPosition(ticks);
        motors.get(3).setPower(power);
        motors.get(3).setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turnToAngle(float angle, float power, int precision)
    {
        for(int i = 1; i <= precision; i++) {

            while (imu.getRobotYawPitchRollAngles().getYaw() < angle)
            {
                motors.get(0).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(0).setPower(-power/i);

                motors.get(1).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(1).setPower(-power/i);

                motors.get(2).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(2).setPower(power/i);

                motors.get(3).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(3).setPower(power/i);
            }

            while (imu.getRobotYawPitchRollAngles().getYaw() > angle)
            {
                motors.get(0).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(0).setPower(power/i);

                motors.get(1).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(1).setPower(power/i);

                motors.get(2).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(2).setPower(-power/i);

                motors.get(3).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(3).setPower(-power/i);
            }

            for (DcMotor motor : motors)
            {
                motor.setPower(0);
            }
        }
        for (DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void shoot3(float power)
    {
        intake.shoot3(power);
    }
    public void shoot(float power)
    {
        intake.shoot(power);
    }
    public void turnToCenterGoal(float power, int precision)
    {
        aprilTagDetection.turnToCenterGoal(motors, power, precision);
    }
    public void moveToGoalDistance(double distance, float power, int precision)
    {
        aprilTagDetection.moveToGoalDistance(motors, distance, power, precision);
    }
    public float getPow()
    {
        return intake.getPower((float)aprilTagDetection.getDistance());
    }
    public float getPowLinear()
    {
        return intake.getPowLinear((float)aprilTagDetection.getDistance());
    }


}
