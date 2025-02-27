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
    //Gets to position for specimen
//      robot.angularStrafe(90, 1000, 1); //was 100, 600, 1
        robot.driveTo(815, telemetry);

    //Clips the specimen
        robot.jointSet(0.3);
        robot.powerTurn(0.7, 2050, 0.2);
        robot.wait(300);
        robot.powerTurn(0.01, 0, 0.3);
        robot.intakeEnable(0, 333);

    //drive to parking area
        robot.angularStrafe(-45, 1400, 1);
        robot.strafe(1300);
        robot.wait(5000);
//Does a Wheelie:

//        int i = -45;
//        while(i <= 90){
//            robot.angularStrafe(i, 100, 0);
//            i++;
//        }

//        robot.drive(0.0);



        //code end
        stop();
    }

}
