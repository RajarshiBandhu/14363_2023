package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.Wheels;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    Wheels wheels;
    double mul;
    @Override
    public void init() {
        mul = 0.6;
    }
    @Override
    public void loop() {


        wheels.fieldCentric(mul);
        if(gamepad1.dpad_up) {
            wheels.resetIMU();
        }

    }
}
