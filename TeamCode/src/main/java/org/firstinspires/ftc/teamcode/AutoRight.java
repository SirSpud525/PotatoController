package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoRight", group = "Right")
public class AutoRight extends LinearOpMode {

  PotatoRobot robot = new PotatoRobot();

  public void runOpMode() throws InterruptedException {
    robot.init(hardwareMap);
    waitForStart();
    robot.driveToInches(6, telemetry);
    robot.strafe(24);
    stop();
  }
}


// Move forward 6 inches, strafe 24 inches
