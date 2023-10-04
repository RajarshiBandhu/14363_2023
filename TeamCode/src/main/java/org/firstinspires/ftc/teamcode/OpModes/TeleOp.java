package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.Wheels;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private Gamepad Driver, Operator;
    Wheels wheels;
    @Override
    public void init()
    {
        Driver = new Gamepad();

    }
    @Override
    public void loop() {
        wheels.fieldCentric(Driver);
        if(gamepad1.dpad_up) {
            wheels.resetIMU();
        }

    }
}
