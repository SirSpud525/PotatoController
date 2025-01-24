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

        robot.angularStrafe(135, 1300, 1);
        robot.driveTo(225, telemetry);

//      robot.angularStrafe(0, 0, 1);
        robot.jointSet(0.2);
        robot.powerTurn(0.65, 1850);



        //        robot.driveToInches(55, telemetry);
//        for(i == -90; i<=0; i++;){
//            robot.angularStrafe(i, 10, 0);
//        }

//        for(i == 180; i<=270; i++;){
//            robot.angularStrafe(i, 10, 0);
//        }
//      robot.angularStrafe(0, 0, 1);


        //code end
        stop();
    }

}
