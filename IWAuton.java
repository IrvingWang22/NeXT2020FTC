package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Config
@Autonomous(name = "IWAuton")
public class IWAuton extends LinearOpMode {
    IWRobotObject robot = new IWRobotObject();
    double power;
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        double intakePower = 0.5;

        telemetry.addLine("Robot Ready");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //start at -60,-50
        Pose2d startPose = new Pose2d(-60, -50, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        waitForStart();

        drive.trajectoryBuilder(new Pose2d())
                //Spin Intake Motor at 2 seconds
                .addTemporalMarker(2, () -> {
                    robot.intakeMotor.setPower(intakePower);
                });

        drive.trajectoryBuilder(new Pose2d())
                //Stop Intake Motor at 4 seconds
                .addTemporalMarker(4, () -> {
                    robot.intakeMotor.setPower(0);
                });

        drive.trajectoryBuilder(new Pose2d())
                //Shoot Ring when the robot reaches 12, -60
                .addSpatialMarker(new Vector2d(12, -60), () -> {
                    //shoot ring
                });

        //Create Trajectory to spline to 12,-60
        Trajectory traj1 = drive.trajectoryBuilder(startPose, false)
                .splineTo(new Vector2d(12, -60), Math.toRadians(45))
                .build();
        //Create Trajectory to spline to 36,-36
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), false)
                .splineTo(new Vector2d(36, -36), Math.toRadians(45))
                .build();
        //Create Trajectory to spline to 60,-60
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), false)
                .splineTo(new Vector2d(60, -60), Math.toRadians(45))
                .build();

        //Execute Trajectory
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);


    }
}
