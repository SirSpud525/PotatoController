package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLeftGood", group = "LeftGood")
public class AutoLeft1Basket extends LinearOpMode{

    PotatoRobot robot = new PotatoRobot();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.driveToInches(30, telemetry);
        robot.turn(135);
        robot.strafe(400);
        robot.driveToInches(45, telemetry);
        robot.raiseSlides(-3500);

        stop();
    }

}
