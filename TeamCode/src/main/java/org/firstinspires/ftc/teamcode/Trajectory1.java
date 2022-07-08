package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@TeleOp(name = "trajectory 1",group = "trajectory")
public class Trajectory1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(23,-60-4,Math.toRadians(0));

        drive.setPoseEstimate(start);

        TrajectorySequence BackDrop = drive.trajectorySequenceBuilder(start)
                .back(0.1)
                .splineTo(new Vector2d(-12,-36-8),Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-12,-36-8),Math.toRadians(90))
                .strafeRight(0.1)
                .splineToConstantHeading(new Vector2d(23,-66),Math.toRadians(0))
                .build();

        waitForStart();

        while(opModeIsActive())
        {
            drive.followTrajectorySequence(BackDrop);
        }
    }
}
