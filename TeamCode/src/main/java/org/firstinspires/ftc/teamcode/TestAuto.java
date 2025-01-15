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
        robot.driveTo(12, telemetry);
        robot.turn(90);
        robot.raiseSlides(-300);
        robot.driveToInches(1000, telemetry);

        //code ends
        stop();
    }

}
