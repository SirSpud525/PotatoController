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
        robot.strafe(525);
        robot.driveToInches(35, telemetry);
        robot.raiseSlides(-2600);
        robot.powerTurn(0.6, 1600);
        robot.intakeEnable(-1, 2000);
        robot.powerTurn(-0.6, 1600);
        robot.raiseSlides(2400);
//

        stop();
    }

}
