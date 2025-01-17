package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MyTeleOp", group = "TeleOp")
public class MyTeleOp extends OpMode {
    PotatoRobot robot = new PotatoRobot();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    boolean lastPass = false;
    boolean toggle = false;

    @Override
    public void loop() {

        if (gamepad1.y && !lastPass){
            toggle = !toggle;
        }

        lastPass = gamepad1.y;

        robot.gamePadPower(gamepad1, gamepad2, telemetry, toggle);
    }
}
