package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

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
private DcMotor slide1;
private DcMotor slide2;
private DcMotor arm;
private CRServo intake;
private CRServo intake2;
private Servo joint;
public IMU imu;

    //potato
//mashed potatoes
    //var

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        slide1 = hardwareMap.get(DcMotor.class, "armMotor");
        slide2 = hardwareMap.get(DcMotor.class, "armMover");
        intake = hardwareMap.get(CRServo.class, "claw");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
        joint = hardwareMap.get(Servo.class, "joint");
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Set reverse motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        slide1.setDirection(DcMotor.Direction.REVERSE);
        //encoders
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setTargetPosition(0);
        backLeft.setTargetPosition(0);
        backRight.setTargetPosition(0);
        frontRight.setTargetPosition(0);

        //zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        joint.setPosition(1.0);

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
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

    public void potatoDrive (Gamepad gp1, Telemetry telemetry){
        //gyro driving function
        telemetry.addData("IMU-X", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).firstAngle);
        telemetry.addData("IMU-Y", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).secondAngle);
        telemetry.addData("IMU-Z", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, DEGREES).thirdAngle);

        if (gp1.x == true){
            imu.resetYaw();
        }

        double imuPos = (imu.getRobotYawPitchRollAngles().getYaw(RADIANS));
        telemetry.addData("IMU-Angle", imuPos);

        double change = Math.cos(-imuPos);
        double sine = Math.sin(-imuPos);

        double drive = (-gp1.left_stick_y);
        double strafe = (gp1.left_stick_x);
        double turn = (gp1.right_stick_x);

//        telemetry.addData("drive", drive);
//        telemetry.addData("strafe", strafe);
//        telemetry.addData("turn", turn);

//        flDrivePower = ((-Math.sin(angle + (0.25 * Math.PI)) * magnitude) + turn);
//        frDrivePower = ((Math.sin(angle - (0.25 * Math.PI)) * magnitude) + turn);
//        blDrivePower = ((-Math.sin(angle - (0.25 * Math.PI)) * magnitude) + turn);
//        brDrivePower = ((Math.sin(angle + (0.25 * Math.PI)) * magnitude) + turn);

        double driveCos = change * drive;
        double driveSin = sine * drive;
        double strafeSin = sine * strafe;
        double strafeCos = change * strafe;

        double actualDrive = (driveCos + strafeSin) / 1.5;
        double actualStrafe = -driveSin + strafeCos;

        flDrivePower = (actualDrive + actualStrafe + turn);
        frDrivePower = (actualDrive - actualStrafe - turn);
        blDrivePower = (actualDrive - actualStrafe + turn);
        brDrivePower = (actualDrive + actualStrafe - turn);

//        telemetry.addData("fl", flDrivePower);
//        telemetry.addData("fr", frDrivePower);
//        telemetry.addData("bl", blDrivePower);
//        telemetry.addData("br", brDrivePower);
//        telemetry.update();

        double slowdown = 0.85;

        if (gp1.right_trigger >= 0.1){
            slowdown = 0.25;
        } else if (gp1.left_trigger >= 0.1) {
            slowdown = 1.25;
        }

        frontLeft.setPower(flDrivePower * slowdown);
        frontRight.setPower(frDrivePower * slowdown);
        backLeft.setPower(blDrivePower * slowdown);
        backRight.setPower(brDrivePower * slowdown);

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

//    public void armMovement (Gamepad gp2){
//        final double armPower = (gp2.right_stick_y);
//        final double armTurn = (gp2.left_stick_y);
//
//        armMotor.setPower(armPower);
//
//        double absolute = Math.abs(armTurn);
//        double sign = armTurn / absolute;
//
//        if (absolute <= 0.1) {
//            armMover.setPower(0.0);
//        } else {
//            armMover.setPower(sign * 0.6);
//        }
//    }

    public void jointOn(Gamepad gp2) {
        if (gp2.left_bumper || gp2.right_bumper) {
            if (gp2.left_bumper){
                joint.setPosition(1.0);
            } else {
                joint.setPosition(0.0);
            }
        }
    }

// raises the arm!!!
    public void raiseArm(Gamepad gp2, Telemetry telemetry){

        double armPwr = 0.63;

        if (gp2.x){
            armPwr = 0.73;
        }
        if (gp2.b){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        double rightStick_y = armPwr *(Math.abs(gp2.right_stick_y)/gp2.right_stick_y);

        if(Math.abs(gp2.right_stick_y) > 0.1){
//          arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setPower(rightStick_y); // makes the robot arm go up and the "gp2.right_stick" is to slow it down
        } else if (gp2.dpad_down){
            arm.setPower(-0.27);
        } else if (gp2.dpad_up){
            arm.setPower(0.27);
        } else {
//            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            arm.setPower(-0.3 * Math.sin((arm.getCurrentPosition() + 1040) / 330));
            arm.setPower((-1 * ((0.0005479 * arm.getCurrentPosition()) + 0.2986)) / 2);
            }
//        telemetry.addData("armPow", -0.3 * Math.sin((arm.getCurrentPosition() + 1040) / 330));
        telemetry.addData("armPow", ((-1* ((0.0005479 * arm.getCurrentPosition()) + 0.2986))) / 2);
//        telemetry.addData("armPwr", armPwr);
        telemetry.addData("armPos", arm.getCurrentPosition());
        telemetry.update();

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

    public void slideMovement(Gamepad gp2, boolean toggle2)
    {

    double slidePower = (gp2.left_stick_y); // Gets the left stick y position of controller
    double slideAbs = Math.abs(slidePower);
    double slideSign = (slidePower/slideAbs);
    double pwr = -0.1;
//was -0.1 no encoder

    if (toggle2) {
        pwr = 0.6;
    }

    //decides how much power to give slides
    if (gp2.y == true){
        slide1.setPower(0.0);
        slide2.setPower(0.0);
    }else {
        if (slideAbs >= 0.1) // moves Linear Slide twins move up or down
        {
            slide1.setPower(slideSign * 0.6);
            slide2.setPower(slideSign * 0.6);
        } else // pwr is passive power to keep the slides where they need to be
        {
            slide1.setPower(pwr);
            slide2.setPower(pwr);
        }
    }
    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2, Telemetry telemetry, boolean toggle, boolean toggle2) {
        if (toggle == true){
            Driving(gp1, telemetry);
        } else {
            potatoDrive(gp1, telemetry);
        }
        slideMovement(gp2, toggle2);
        clawClawing(gp2);
        jointOn(gp2);
        raiseArm(gp2, telemetry);
//        telemetry.addData("slides", slide1.getCurrentPosition());
    }

    public void drive(final double pow){
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
        final int THRESHOLD = 35;
        double ADDITIONAL_SPEED;

        if (pos >= 0){
            ADDITIONAL_SPEED = 0.1;
        }else {
            ADDITIONAL_SPEED = -0.1;
        }

        while (Math.abs(pos - frontLeft.getCurrentPosition()) > THRESHOLD || Math.abs(pos - frontRight.getCurrentPosition()) > THRESHOLD) {
            int flDistance = pos - frontLeft.getCurrentPosition();
            int frDistance = pos - frontRight.getCurrentPosition();
            int blDistance = pos - backLeft.getCurrentPosition();
            int brDistance = pos - backRight.getCurrentPosition();

            ADDITIONAL_SPEED = (flDistance / Math.abs(flDistance)) * 0.1;

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

//            slide1.setPower(-0.1);
//            slide2.setPower(-0.1);
            try {
                Thread.sleep(delay);
            } catch (InterruptedException e) {
            }
        }

        drive(0.0);
    }

    public void raiseSlides (double amt){
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double additional = -0.15;
        double sign = -1;
        double distance = 100;

        if (amt >= 0){
            additional = 0.1;
            sign = 1;
        }

        while (Math.abs(amt - slide1.getCurrentPosition()) > distance){
            double toGo = amt - slide1.getCurrentPosition();
            double slidePow;

            if (toGo < 175){
                slidePow = toGo/Math.abs(amt);
            } else {
                slidePow = sign * 0.85;
            }

            slide1.setPower((slidePow / 2) + additional);
            slide2.setPower((slidePow / 2) + additional);
        }

        slide1.setPower(-0.0031415926535897932384626);
        slide2.setPower(-0.0031415926535897932384626);

    }

    public void armMove (double amt){
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double distance = 100;
        double additional = 0.1;

        while (Math.abs(amt - arm.getCurrentPosition()) > distance){
            double toGo = amt - arm.getCurrentPosition();

            double slidePow = toGo/Math.abs(amt);

            arm.setPower((slidePow / 2) + additional);
        }

    }

    public void jointSet(double AMOUNT){

        joint.setPosition(AMOUNT);
//try{
//        Thread.sleep(10000);} catch (InterruptedException e) {
//        }
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
//kp was .5
        double kp = 1;

        final int DELAY = 50;

        while (Math.abs(error) > 2) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180 * kp);
            final double mult = 2.4;

            flDrivePower = -turn * mult;
            frDrivePower = turn * mult;
            blDrivePower = -turn * mult;
            brDrivePower = turn * mult;

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
            frontRight.setPower(frDrivePower / 2 - extra);
            backLeft.setPower(blDrivePower / 2 - extra);
            backRight.setPower(brDrivePower / 2 + extra);

//            slide1.setPower(-0.1);
//            slide2.setPower(-0.1);
        }

        drive(0.0);

        try {Thread.sleep(50);} catch (InterruptedException e) {}
    }

public void intakeEnable(double rotate, final int milliseconds){ //0 corresponds to opening, 1 corresponds to closing
        final double rotationType = rotate;

        if (rotationType >= 0.1) {
            intake.setPower(-1);
            intake2.setPower(1);
        } else {
            intake.setPower(1);
            intake2.setPower(-1);
        }

        try {Thread.sleep(milliseconds);} catch (InterruptedException e) {}

       intake.setPower (0.0);
       intake2.setPower (0.0);

    }

    public void wait(int milliseconds){
        try {Thread.sleep(milliseconds);} catch (InterruptedException e) {}
    }

    //next two functions are power based auto, only use in emergency or lack of encoders
//    public void powerArm(double power, int length){
//
//        armMotor.setPower(power);
//
//        try {Thread.sleep(length);} catch (InterruptedException e) {}
//
//        armMotor.setPower(0.0);
//    }
//
    public void powerTurn(double powerT, int lengthT, double passive){
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setPower(powerT);

        try {Thread.sleep(lengthT);} catch (InterruptedException e) {}

        if (powerT >= 0) {
            arm.setPower(-1 * passive);
        } else {
            arm.setPower(passive);
        }
    }

    private void enableAllMotors(final double p1, final double p2){ //abstraction
        frontRight.setPower(p2);
        backRight.setPower(p1);
        frontLeft.setPower(p1);
        backLeft.setPower(p2);
    }

    //test strafe func, strafes based on starting pos (or imu var)
    public void angularStrafe(int degrees, int distance, int breakONend) {

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double xDist = 1000;
        double yDist = 1000;
        final double error = 100;
        final double slowdown = 1.8;

        while (Math.abs(yDist - frontRight.getCurrentPosition()) > error || Math.abs(xDist - frontLeft.getCurrentPosition()) > error) {

            double Pitch = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES); //makes sure that it strafes relative to the starting IMU pos
            double angleInRadians = Math.toRadians(degrees + 315 - Pitch); //0 deg, would strafe right, 90 deg would strafe up, etc.
            double cosValue = Math.cos(angleInRadians);
            double sinValue = Math.sin(angleInRadians);
            yDist = distance * sinValue;
            xDist = distance * cosValue;

            double x = cosValue;
            double y = sinValue;

            enableAllMotors(x / slowdown, y / slowdown);
        }
    if (breakONend == 1){
        drive(0.0);
    }

    }

    public void fastTurn(int deg){
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.imu.resetYaw();
        double pos = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);

        while(Math.abs(deg - pos) > 15){
            pos = this.imu.getRobotYawPitchRollAngles().getYaw(DEGREES);

            if (deg - pos >= 0){
                frontLeft.setPower(0.8);
                frontRight.setPower(-0.8);
                backRight.setPower(-0.8);
                backLeft.setPower(0.8);
            } else {
                frontLeft.setPower(-0.8);
                frontRight.setPower(0.8);
                backRight.setPower(0.8);
                backLeft.setPower(-0.8);
            }

        }

        turn(deg - pos);

    }

}

