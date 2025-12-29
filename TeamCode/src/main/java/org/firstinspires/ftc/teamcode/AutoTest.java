package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;

@Autonomous
public class AutoTest extends LinearOpMode {

    public void runOpMode()
    {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        List<DcMotor> motors = List.of(frontLeftMotor,backLeftMotor,frontRightMotor,backRightMotor);

        for(DcMotor motor : motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //unique lines start here

        moveBackward(motors, 490, .5f);
        turnToAngle(motors, imu, 330f, 0.5f, 5);
        moveLeft(motors, 244, 0.5f);

        //moveForward(motors, 1500, 0.5f);
    }

    public void moveForward(List<DcMotor> motors, int ticks, float power)
    {
        for (DcMotor motor : motors)
        {
            motor.setTargetPosition(ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void moveBackward(List<DcMotor> motors, int ticks, float power)
    {
        for (DcMotor motor : motors)
        {
            motor.setTargetPosition(-ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void moveLeft(List<DcMotor> motors, int ticks, float power)
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
    public void moveRight(List<DcMotor> motors, int ticks, float power)
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

    public void turnToAngle(List<DcMotor> motors, IMU imu, float angle, float power, int precision)
    {
        for(int i = 0; i < precision; i++) {

            while (imu.getRobotYawPitchRollAngles().getYaw() < angle)
            {
                motors.get(0).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(0).setPower(-power);

                motors.get(1).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(1).setPower(-power);

                motors.get(2).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(2).setPower(power);

                motors.get(3).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(3).setPower(power);
            }

            for (DcMotor motor : motors)
            {
                motor.setPower(0);
            }

            while (imu.getRobotYawPitchRollAngles().getYaw() < angle)
            {
                motors.get(0).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(0).setPower(power);

                motors.get(1).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(1).setPower(power);

                motors.get(2).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(2).setPower(-power);

                motors.get(3).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.get(3).setPower(-power);
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
}
