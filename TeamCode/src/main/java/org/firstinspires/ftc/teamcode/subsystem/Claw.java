package org.firstinspires.ftc.teamcode.subsystem;


import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw implements Subsystem{
    private Servo leftHand, rightHand, wrist;

    public Claw(HardwareMap hardwareMap){
        leftHand = hardwareMap.servo.get("leftHand");
        rightHand = hardwareMap.servo.get("rightHand");
        wrist = hardwareMap.servo.get("wrist");
    }


}
