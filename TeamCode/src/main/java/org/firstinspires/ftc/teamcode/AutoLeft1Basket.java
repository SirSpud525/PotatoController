package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLeftGood", group = "LeftGood")
public class AutoLeft1Basket extends LinearOpMode{

    PotatoRobot robot = new PotatoRobot();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

//two basket (hopefully)

        robot.jointSet(0.7);//originally 0.7

        robot.strafe(150);
        robot.driveToInches(30, telemetry);
        robot.angularStrafe(45, 250, 1);
        robot.turn(45);
        robot.strafe(350);
        robot.driveToInches(8, telemetry);

        robot.raiseSlides(-2500);
        robot.powerTurn(0.75, 1800, 0.25);
        robot.intakeEnable(0, 1000);
        robot.powerTurn(-0.7, 1600, 0.2);
        robot.raiseSlides(1900);

        robot.turn(-135);
        robot.driveToInches(50, telemetry);
        robot.strafe(285);

//        robot.raiseSlides(-600);
        robot.jointSet(0.0);
        robot.powerTurn(0.7, 1300, 0.2);
        robot.wait(500);
        robot.powerTurn(0.01, 0, 0.0);
        robot.raiseSlides(725);
        robot.intakeEnable(1, 750);

        robot.wait(5000);

//one basket below

//        robot.driveToInches(30, telemetry);
//        robot.jointSet(0.7);
//        robot.turn(135);
//        robot.strafe(525);
//        robot.driveToInches(35, telemetry);
//        robot.raiseSlides(-2600);
//        robot.powerTurn(0.6, 1600, 0.2);
//        robot.intakeEnable(-1, 2000);
//        robot.powerTurn(-0.6, 1600, 0.2);
//        robot.raiseSlides(2400);
//        robot.turn(-137);
//mashed potatoes

        stop();
    }

}
