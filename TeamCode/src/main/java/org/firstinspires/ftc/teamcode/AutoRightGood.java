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
        robot.jointSet(-1);
//        robot.driveToInches(55, telemetry);
//        robot.jointSet(-0.5);
//        robot.strafe(-925);
//        robot.jointSet(-0.5);
//        robot.powerTurn(0.6, 1300);
//        robot.jointSet(-0.5);
//        robot.powerTurn(-0.6, 1100);
//        robot.jointSet(-0.5);

//        robot.strafe(-1900);
//        robot.driveToInches(96, telemetry);
//        robot.turn(-90);

        //code end
//        stop();
    }

}
