package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TeleOp(){

    Gamepad gamepad1 = new Gamepad();
    Gamepad gamepad2 = new Gamepad();

    PotatoRobot robot = new PotatoRobot();
    public void loop() {
        robot.gamePadPower(gamepad1, gamepad2);

    }
}
