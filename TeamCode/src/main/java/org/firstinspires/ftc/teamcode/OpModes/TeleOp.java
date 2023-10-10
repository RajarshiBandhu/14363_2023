package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.Wheels;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private GamepadEx Driver;

    private Wheels driveTrain;
    Wheels wheels;
    @Override
    public void init()
    {
        Driver = new GamepadEx(gamepad1);
        driveTrain = new Wheels(hardwareMap);
    }
    @Override
    public void loop() {
        Driver.readButtons();

        driveTrain.fieldCentric(Driver);
        if(gamepad1.dpad_up) {
            wheels.resetIMU();
        }

    }
}
