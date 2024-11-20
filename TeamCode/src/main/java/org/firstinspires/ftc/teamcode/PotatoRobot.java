package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
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


public class PotatoRobot {
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
public IMU imu;

    //potato

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMover = hardwareMap.get(DcMotor.class, "armMover");
        intake = hardwareMap.get(CRServo.class, "claw");

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
    }

    public void Driving(Gamepad gp1){
        double multiplier = Math.max(0.3, 1 - gp1.right_trigger);

        final double drive = (-gp1.left_stick_y);
        final double turn = (gp1.right_stick_x);
        final double strafe = (gp1.left_stick_x);

        flDrivePower = (drive + strafe + turn);
        frDrivePower = (drive - strafe - turn);
        blDrivePower = (drive - strafe + turn);
        brDrivePower = (drive + strafe - turn);

        frontLeft.setPower(flDrivePower * multiplier / 1.5);
        frontRight.setPower(frDrivePower * multiplier / 1.5);
        backLeft.setPower(blDrivePower * multiplier / 1.5);
        backRight.setPower(brDrivePower * multiplier / 1.5);
    }

    public void armMovement (Gamepad gp2){
        final double armPower = (gp2.right_stick_y);
        final double armTurn = (gp2.left_stick_y);

        armMotor.setPower(armPower);

        double absolute = Math.abs(armTurn);
        double sign = armTurn / absolute;

        if (absolute <= 0.1) {
            armMover.setPower(0.0);
        } else {
            armMover.setPower(sign * 0.35);
        }
    }

    public void clawClawing(Gamepad gp2){

        final double intakeOn = (gp2.right_trigger);
        final double intakeR = (gp2.left_trigger);

        if (intakeOn >= 0.1){
            if (intakeR >= 0.1){
                intake.setPower (0.0);
            } else {
                intake.setPower (0.3);
            }
        } else {
            if (intakeR >= 0.1){
                intake.setPower (-0.3);
            } else {
                intake.setPower (0.0);
            }
        }

    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2) {
        Driving(gp1);
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
    public void strafe(int pos) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int flBrPos = pos;
        int frBlPos = -pos;

        while (Math.abs(flBrPos - backRight.getCurrentPosition()) > 10 || Math.abs(flBrPos - frontLeft.getCurrentPosition()) > 10) {
            int flDistance = flBrPos - frontLeft.getCurrentPosition();
            int frDistance = frBlPos - frontRight.getCurrentPosition();
            int blDistance = frBlPos - backLeft.getCurrentPosition();
            int brDistance = flBrPos - backRight.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(flBrPos);
            blDrivePower = (double)blDistance / (double)Math.abs(frBlPos);
            frDrivePower = (double)frDistance / (double)Math.abs(frBlPos);
            brDrivePower = (double)brDistance / (double)Math.abs(flBrPos);

            frontLeft.setPower(flDrivePower / 5);
            frontRight.setPower(frDrivePower / 5);
            backLeft.setPower(blDrivePower / 5);
            backRight.setPower(brDrivePower / 5);
        }

        drive(0.0);

        try {Thread.sleep(500);} catch (InterruptedException e) {}
    }
}
