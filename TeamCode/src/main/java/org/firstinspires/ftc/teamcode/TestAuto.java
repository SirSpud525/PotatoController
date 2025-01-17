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



        //code ends
        stop();
    }

}

//donut
//
//   for (int i = 0; i < 360; i++ ){
//        robot.angularStrafe(45, 50);
//        robot.turn(-1);
//        }