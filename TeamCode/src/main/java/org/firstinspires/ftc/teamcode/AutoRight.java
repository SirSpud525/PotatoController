package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoRight", group = "Right")
public class AutoRight extends LinearOpMode {

  PotatoRobot robot = new PotatoRobot();

  public void runOpMode() throws InterruptedException {
    robot.init(hardwareMap);
    waitForStart();
    robot.driveToInches(30, telemetry);
    robot.strafe(1150);
    robot.driveToInches(-30, telemetry);
    stop();
  }
}


// Move forward 6 inches, strafe 24 inches
