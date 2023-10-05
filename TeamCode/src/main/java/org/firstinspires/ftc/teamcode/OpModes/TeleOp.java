package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.Wheels;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode
{
    private Gamepad Driver = new Gamepad(), Operator;
    Wheels wheels;
    @Override
    public void init()

    {

    }
    @Override
    public void loop() {
        wheels.fieldCentric(Driver);
        if(gamepad1.dpad_up) {
            wheels.resetIMU();
        }

    }
}
