package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoRedLeft", group = "Left")
public class AutoRedLeft extends LinearOpMode {

  PotatoRobot robot = new PotatoRobot();

  public void runOpMode() throws InterruptedException {
    robot.init(hardwareMap);
    waitForStart();
    robot.driveToInches(6, telemetry);
    robot.strafe(56);
    stop();
  }
//move foward 6 in, strafe right 96 in

  //Potatoes are bad
}
