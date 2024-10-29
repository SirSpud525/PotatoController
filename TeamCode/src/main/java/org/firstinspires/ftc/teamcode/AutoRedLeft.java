package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "AutoRedLeft", group = "Left")
public class AutoRedLeft extends OpMode {

  PotatoRobot robot = new PotatoRobot();

  public void init() {
    robot.init(hardwareMap);
  }

  @Override
  public void loop() {
    robot.driveToInches(6);
    robot.strafe(96);

  }
//move foward 6 in, strafe right 96 in

}
