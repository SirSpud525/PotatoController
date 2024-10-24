package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public IMU imu;

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set up drive motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        frontLeft.setPower(flDrivePower * multiplier / 4.0);
        frontRight.setPower(frDrivePower * multiplier / 4.0);
        backLeft.setPower(blDrivePower * multiplier / 4.0);
        backRight.setPower(brDrivePower * multiplier / 4.0);
    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2) {
    Driving(gp1);

    

    }
//trfP

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
    public void driveToInches(final double inches) {
        driveTo((int) (inches * (100 / 11.75) * 1.5));
    }

    public void driveTo(final int pos) {
        if (pos == 0) return;

        this.imu.resetYaw();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        final int delay = 20;

        while (Math.abs(pos - frontLeft.getCurrentPosition()) > 10 || Math.abs(pos - frontRight.getCurrentPosition()) > 10) {
            int flDistance = pos - frontLeft.getCurrentPosition();
            int frDistance = pos - frontRight.getCurrentPosition();
            int blDistance = pos - backLeft.getCurrentPosition();
            int brDistance = pos - backRight.getCurrentPosition();

            flDrivePower = (double) flDistance / (double) Math.abs(pos);
            blDrivePower = (double) blDistance / (double) Math.abs(pos);
            frDrivePower = (double) frDistance / (double) Math.abs(pos);
            brDrivePower = (double) brDistance / (double) Math.abs(pos);

            frontLeft.setPower(flDrivePower / 3);
            frontRight.setPower(frDrivePower / 3);
            backLeft.setPower(blDrivePower / 3);
            backRight.setPower(brDrivePower / 3);

            try {
                Thread.sleep(delay);
            } catch (InterruptedException e) {
            }
        }
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
