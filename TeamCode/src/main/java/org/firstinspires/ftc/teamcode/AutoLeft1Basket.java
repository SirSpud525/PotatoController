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

//        robot.jointSet(0.7);

//        robot.strafe(150);
//        robot.driveToInches(30, telemetry);
//        robot.angularStrafe(45, 250, 1);
//        robot.turn(45);
//        robot.strafe(350);
//        robot.driveToInches(10, telemetry);

        robot.raiseSlides(-2300);
        robot.powerTurn(0.6, 1800, 0.2);
        robot.intakeEnable(0, 1000);

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
//

        stop();
    }

}
