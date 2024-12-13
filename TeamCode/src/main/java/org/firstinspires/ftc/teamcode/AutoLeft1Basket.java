package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLeftGood", group = "LeftGood")
public class AutoLeft1Basket extends LinearOpMode{

    PotatoRobot robot = new PotatoRobot();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

//        robot.driveToInches(30, telemetry);
//        robot.turn(-55);
//        robot.strafe(-360);
//        robot.driveToInches(5, telemetry);
//        robot.powerTurn(-0.6, 1000);
//        robot.powerArm(-0.7, 4975);
//        robot.intakeEnable(0, 3000);
//        robot.powerArm(0.7, 4500);
//        robot.turn(55);
        stop();
    }

}
