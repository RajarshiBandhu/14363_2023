package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Wheels
{
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    public Wheels(HardwareMap hardwareMap)
    {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotorEx.class,"frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotorEx.class,"backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotorEx.class,"frontRightMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"backRightMotor");

        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "cIMU");
        // Makes a new object titled 'parameters' usd to hold the angle of the IMU
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }

    public void resetIMU()
    {
        imu.initialize(parameters);
    }

    public void fieldCentric(GamepadEx gamepad1){
        double y = gamepad1.getLeftY();
        double x = -gamepad1.getLeftX();
        double rx = -gamepad1.getRightX();
        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = 1 * (rotY + rotX + rx) / denominator;
        double backLeftPower = 1 * (rotY - rotX + rx) / denominator;
        double frontRightPower = 1 * (rotY - rotX - rx) / denominator;
        double backRightPower = 1 * (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

}
