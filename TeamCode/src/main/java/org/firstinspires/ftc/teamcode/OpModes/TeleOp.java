package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystem.mecanum;

public class TeleOp extends OpMode {
    mecanum wheels;
    @Override
    public void init() {
        double mul = 1.1;
        wheels.fieldCentric(mul);
    }

    @Override
    public void loop() {

        if(gamepad1.dpad_up) {
            wheels.resetIMU();
        }

    }
}
