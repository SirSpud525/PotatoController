package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "AutoRightGood", group = "RightGood")

public class AutoRightGood extends LinearOpMode{

    PotatoRobot robot = new PotatoRobot();

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        //code start
        robot.driveToInches(8,telemetry);
        robot.strafe(-1700);
        robot.driveToInches(39, telemetry);
        robot.raiseSlides(-500);
        robot.raiseSlides(300);
        robot.strafe(-1900);
        robot.driveToInches(96, telemetry);
        robot.turn(-90);
        //code end
        stop();
    }

}
