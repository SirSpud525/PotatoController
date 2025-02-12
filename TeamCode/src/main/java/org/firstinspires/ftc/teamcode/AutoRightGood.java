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

//      robot.angularStrafe(90, 1085, 1);
        robot.driveTo(825, telemetry);
        robot.jointSet(0.3);
        robot.powerTurn(0.65, 1900, 0.2);
        robot.wait(200);
        robot.powerTurn(0.01, 0, 0.3);

        robot.intakeEnable(0, 333);
        robot.angularStrafe(-20, 1100, 1);
        robot.angularStrafe(20, 1100, 1);
        robot.angularStrafe(90, 600,1);
        robot.raiseSlides(200);
//        robot.wait(1000);
//        robot.intakeEnable(0, 333);
//        robot.angularStrafe(-45, 1000, 1);
//        int i = -45;
//        while(i <= 90){
//            robot.angularStrafe(i, 100, 0);
//            i++;
//        }

        robot.drive(0.0);



        //code end
        stop();
    }

}
