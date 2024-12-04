package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class PotatoRobot {
    final String robotName = "Potato";
    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    private DcMotor frontLeft;
private DcMotor frontRight;
private DcMotor backLeft;
private DcMotor backRight;
private DcMotor armMotor;
private DcMotor armMover;
private CRServo intake;
private CRServo intake2;
public IMU imu;

    //potato

    //var

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMover = hardwareMap.get(DcMotor.class, "armMover");
        intake = hardwareMap.get(CRServo.class, "claw");
        intake2 = hardwareMap.get(CRServo.class, "intake2");

        // Set up drive motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //encoders
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);
        frontRight.setTargetPosition(0);

        //zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);

        this.imu.resetYaw();
    }

    public void Driving(Gamepad gp1, Telemetry telemetry){
        double multiplier = Math.max(0.3, 1 - gp1.right_trigger);
        double speedMode = (gp1.left_trigger);

        double slowdown = 1.5;
        final double drive = (-gp1.left_stick_y);
        final double turn = (gp1.right_stick_x);
        final double strafe = (gp1.left_stick_x);

        flDrivePower = (drive + strafe + turn);
        frDrivePower = (drive - strafe - turn);
        blDrivePower = (drive - strafe + turn);
        brDrivePower = (drive + strafe - turn);

        telemetry.addData("IMU-X", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).firstAngle);
        telemetry.addData("IMU-Y", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).secondAngle);
        telemetry.addData("IMU-Z", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).thirdAngle);

        double imuPos = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
        telemetry.addData("IMU-Angle", imuPos);
        telemetry.update();

        if (speedMode > 0.1){
            slowdown = 1;
        }

        frontLeft.setPower(flDrivePower * multiplier / slowdown);
        frontRight.setPower(frDrivePower * multiplier / slowdown);
        backLeft.setPower(blDrivePower * multiplier / slowdown);
        backRight.setPower(brDrivePower * multiplier / slowdown);
    }

//    public void potatoesAreBad(Gamepad gp1){ //Antonio y pranavs code
//        final boolean yes = (gp1.a);
//
//        if (yes == true){
//            frontLeft.setPower(0.5);
//        }
//        else {
//            frontLeft.setPower(0.0);
//        }
//    }

    public void armMovement (Gamepad gp2){
        final double armPower = (gp2.right_stick_y);
        final double armTurn = (gp2.left_stick_y);

        armMotor.setPower(armPower);

        double absolute = Math.abs(armTurn);
        double sign = armTurn / absolute;

        if (absolute <= 0.1) {
            armMover.setPower(0.0);
        } else {
            armMover.setPower(sign * 0.6);
        }
    }

    public void clawClawing(Gamepad gp2){

        final double intakeOn = (gp2.right_trigger);
        final double intakeR = (gp2.left_trigger);

        if (intakeOn >= 0.1){
            if (intakeR >= 0.1){
                intake.setPower (0.0);
                intake2.setPower(0.0);
            } else {
                intake.setPower (-0.3);
                intake2.setPower(0.3);
            }
        } else {
            if (intakeR >= 0.1){
                intake.setPower (0.3);
                intake2.setPower(-0.3);
            } else {
                intake.setPower (0.0);
                intake2.setPower(0.0);
            }
        }

    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2, Telemetry telemetry) {
        Driving(gp1, telemetry);
        armMovement(gp2);
        clawClawing(gp2);
    }

    private void drive(final double pow){
        frontLeft.setPower(pow);
        backLeft.setPower(pow);
        frontRight.setPower(pow);
        backRight.setPower(pow);
    }

    private void setDriveMode(final DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    public void driveToInches (final double inches, Telemetry telemetry) {
        driveTo((int) (inches * (100 / 11.75) * 1.5), telemetry);
    }

    public void driveTo(final int pos, Telemetry telemetry) {
        if (pos == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final int delay = 20;
        final int THRESHOLD = 20;
        final double ADDITIONAL_SPEED = 0.1;

        while (Math.abs(pos - frontLeft.getCurrentPosition()) > THRESHOLD || Math.abs(pos - frontRight.getCurrentPosition()) > THRESHOLD) {
            int flDistance = pos - frontLeft.getCurrentPosition();
            int frDistance = pos - frontRight.getCurrentPosition();
            int blDistance = pos - backLeft.getCurrentPosition();
            int brDistance = pos - backRight.getCurrentPosition();

            telemetry.addData("dist", flDistance);
            telemetry.update();

            flDrivePower = (double) flDistance / (double) Math.abs(pos);
            blDrivePower = (double) blDistance / (double) Math.abs(pos);
            frDrivePower = (double) frDistance / (double) Math.abs(pos);
            brDrivePower = (double) brDistance / (double) Math.abs(pos);

            frontLeft.setPower(flDrivePower / 3 + ADDITIONAL_SPEED);
            frontRight.setPower(frDrivePower / 3 + ADDITIONAL_SPEED);
            backLeft.setPower(blDrivePower / 3 + ADDITIONAL_SPEED);
            backRight.setPower(brDrivePower / 3 + ADDITIONAL_SPEED);

            try {
                Thread.sleep(delay);
            } catch (InterruptedException e) {
            }
        }

        drive(0.0);
    }

//    public void turn(final int posT) {
//        if (posT == 0) return;
//
//        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        final int delay = 20;
//        final int THRESHOLD = 20;
//        final double ADDITIONAL_SPEED = 0.1;
//
//        while (Math.abs(posT - frontLeft.getCurrentPosition()) > THRESHOLD || Math.abs(posT - frontRight.getCurrentPosition()) > THRESHOLD) {
//            int flDistance = posT - frontLeft.getCurrentPosition();
//            int frDistance = posT - frontRight.getCurrentPosition();
//            int blDistance = posT - backLeft.getCurrentPosition();
//            int brDistance = posT - backRight.getCurrentPosition();
//
//            flDrivePower = (double) flDistance / (double) Math.abs(posT);
//            blDrivePower = (double) blDistance / (double) Math.abs(posT);
//            frDrivePower = (double) frDistance / (double) Math.abs(posT);
//            brDrivePower = (double) brDistance / (double) Math.abs(posT);
//
//            frontLeft.setPower(flDrivePower / 3 + ADDITIONAL_SPEED);
//            frontRight.setPower((frDrivePower / 3 + ADDITIONAL_SPEED) * -1);
//            backLeft.setPower((blDrivePower / 3 + ADDITIONAL_SPEED) * -1);
//            backRight.setPower(brDrivePower / 3 + ADDITIONAL_SPEED);
//
//            try {
//                Thread.sleep(delay);
//            } catch (InterruptedException e) {
//            }
//        }
//    }


    public void turn(double target) {
        this.imu.resetYaw();

        if (target == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
        double error = target - currentPosition;

        double kp = 0.5;

        final int DELAY = 50;

        while (Math.abs(error) > 2) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180 * kp);

            flDrivePower = -turn;
            frDrivePower = turn;
            blDrivePower = -turn;
            brDrivePower = turn;

            frontLeft.setPower(flDrivePower / 2);
            frontRight.setPower(frDrivePower / 2);
            backLeft.setPower(blDrivePower / 2);
            backRight.setPower(brDrivePower / 2);

            try {Thread.sleep(DELAY);} catch (InterruptedException e) {}
        }

        drive(0.0);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {Thread.sleep(50);} catch (InterruptedException e) {}
    }




    public void strafe(int pos) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int flBrPos = pos;
        int frBlPos = -pos;
        double extra = 0.1;
        final double tickDistance = 40;

        if (pos > 0){
            extra = 0.1;
        } else {
            extra = -0.1;
        }

        while (Math.abs(flBrPos - backRight.getCurrentPosition()) > tickDistance || Math.abs(flBrPos - frontLeft.getCurrentPosition()) > tickDistance) {
            int flDistance = flBrPos - frontLeft.getCurrentPosition();
            int frDistance = frBlPos - frontRight.getCurrentPosition();
            int blDistance = frBlPos - backLeft.getCurrentPosition();
            int brDistance = flBrPos - backRight.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(flBrPos);
            blDrivePower = (double)blDistance / (double)Math.abs(frBlPos);
            frDrivePower = (double)frDistance / (double)Math.abs(frBlPos);
            brDrivePower = (double)brDistance / (double)Math.abs(flBrPos);

            frontLeft.setPower(flDrivePower / 2 + extra);
            frontRight.setPower(frDrivePower / 2 + extra);
            backLeft.setPower(blDrivePower / 2 + extra);
            backRight.setPower(brDrivePower / 2 + extra);
        }

        drive(0.0);

        try {Thread.sleep(50);} catch (InterruptedException e) {}
    }

    public void armTurn(double turn){

        final double tickDist = 15;
        final double extraArm = 0.5;

while (Math.abs(turn - armMover.getCurrentPosition()) > tickDist){
    double distance = turn - armMover.getCurrentPosition();
    double armMoverPower = distance/Math.abs(turn);
    armMover.setPower(armMoverPower / 1.75 + extraArm);
}

    }

    public void armExtend(double extend){

        final double tickErr = 15;
        final double extraExtend = 0.5;

        while (Math.abs(extend - armMover.getCurrentPosition()) > tickErr){
            double distance = extend - armMover.getCurrentPosition();
            double armMoverPower = distance/Math.abs(extend);
            armMover.setPower(armMoverPower / 1.75 + extraExtend);
        }


    }
//MASHED POTATOESSSSSSSS
public void intakeEnable(double rotate, final int seconds){ //0 corresponds to opening, 1 corresponds to closing
        final double rotationType = rotate;

        if (rotationType >= 0.1) {
            intake.setPower(-1);
            intake2.setPower(1);
        } else {
            intake.setPower(1);
            intake2.setPower(-1);
        }

        try {Thread.sleep(seconds);} catch (InterruptedException e) {}

       intake.setPower (0.0);
       intake2.setPower (0.0);

    }

//MASHED POTATOES    
    
public void betterTurn(double power, final int seconds){
        final double posPOWER = power;
        final double negPOWER = -1 * power;
        
        frontRight.setPower(posPOWER);
        backRight.setPower(posPOWER);
        frontLeft.setPower(negPOWER);
        backLeft.setPower(negPOWER);

      try {Thread.sleep(seconds * 1000);} catch (InterruptedException e) {}

        frontRight.setPower(0.0);
        backRight.setPower(0.0);
        frontLeft.setPower(0.0);
        backLeft.setPower(0.0);
        
        
}
 
    
//next two functions are power based auto, only use in emergency or lack of encoders
    public void powerArm(double power, int length){

        armMotor.setPower(power);

        try {Thread.sleep(length);} catch (InterruptedException e) {}

        armMotor.setPower(0.0);
    }

    public void powerTurn(double powerT, int lengthT){
        armMover.setPower(powerT);

        try {Thread.sleep(lengthT);} catch (InterruptedException e) {}

        armMover.setPower(0.0);
    }

}
