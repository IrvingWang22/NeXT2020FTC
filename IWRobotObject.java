package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IWRobotObject {
    //Define Hardware
    DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    DcMotor intakeMotor, leftShooterMotor, rightShooterMotor;
    Servo serv1, serv2;

    HardwareMap hardwareMap = null;
    Telemetry telemetry = null;
    public void init(HardwareMap hwMap, Telemetry telemetry) {
        hardwareMap = hwMap;
        telemetry = telemetry;
        //Wheels
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        //Intake
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        //Shooter
        leftShooterMotor = hardwareMap.get(DcMotor.class, "left_shooter_motor");
        rightShooterMotor = hardwareMap.get(DcMotor.class, "right_shooter_motor");
        leftShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        //Servos
        serv1 = hardwareMap.get(Servo.class, "servo_1");
        serv2 = hardwareMap.get(Servo.class, "servo_2");

        telemetry.addLine("Robot Object Ready");
        telemetry.update();

    }
}

