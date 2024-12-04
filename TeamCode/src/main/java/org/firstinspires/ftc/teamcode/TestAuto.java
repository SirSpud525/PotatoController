package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name = "TestAuto", group = "Test")
public class TestAuto extends LinearOpMode{

    PotatoRobot robot = new PotatoRobot();
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        //code here
robot.betterTurn(1.0, 3);
        //code ends
        stop();
    }

}
