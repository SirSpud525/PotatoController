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
    boolean lastPass2 = false;
    boolean toggle2 = false;

    @Override
    public void loop() {

        if (gamepad1.y && !lastPass){
            toggle = !toggle;
        }

        if (gamepad2.a && !lastPass2){
            toggle2 = !toggle2;
        }

        lastPass = gamepad1.y;
        lastPass2 = gamepad2.a;

        robot.gamePadPower(gamepad1, gamepad2, telemetry, toggle, toggle2);
    }
}
