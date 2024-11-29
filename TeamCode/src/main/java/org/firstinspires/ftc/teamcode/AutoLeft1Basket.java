package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLeftGood", group = "LeftGood")
public class AutoLeft1Basket extends LinearOpMode{

    PotatoRobot robot = new PotatoRobot();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.driveToInches(30, telemetry);
        robot.turn(-125);
        robot.driveToInches(10, telemetry);
        robot.powerTurn(50, 1);
        robot.powerArm(50, 3);
        robot.intakeEnable(15, 1);
        stop();
    }


}
