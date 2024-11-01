package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLeft", group = "Left")
public class AutoLeft extends LinearOpMode {

    PotatoRobot robot = new PotatoRobot();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        robot.driveToInches(30, telemetry);
        robot.strafe(4600);
        stop();
    }

}
