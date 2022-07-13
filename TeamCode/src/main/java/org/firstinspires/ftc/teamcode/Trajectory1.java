package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@TeleOp(name = "trajectory 1",group = "trajectory")
public class Trajectory1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Arm and Extension and carousel
    private DcMotorEx arm, extension, carousel, turret;

    Servo servo1,servo2;
    int pos = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");

        //mechanum drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //servos
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");

        // Arm config
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.armpidcoeffs);
//        arm.setPositionPIDFCoefficients(DriveConstants.armpositionpidcoff);
        arm.setVelocityPIDFCoefficients(ArmConstants.vel_p,ArmConstants.vel_i,ArmConstants.vel_d,ArmConstants.vel_f);

        // Extension config
        extension = hardwareMap.get(DcMotorEx.class, "extension");
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.extensionpidcoeffs);

        // Carousel
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        //turret
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Get arm and extension ready
        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setVelocity(ArmConstants.arm_velocity);
        arm.setPower(1);

        extension.setTargetPosition(0);
        extension.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension.setPower(1);

        turret.setTargetPosition(0);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(1);

        int startOffset = 8;

        //setting drive initial pose
        Pose2d start = new Pose2d(24+startOffset,-66,Math.toRadians(0));
        Pose2d shippingHub = new Pose2d(-16,-36-8,Math.toRadians(0));

        drive.setPoseEstimate(start);

        //move arm to uppmost position
        TrajectorySequence PickToTop = drive.trajectorySequenceBuilder(start)
                .addTemporalMarker(() -> {
                    servo1.setPosition(ArmConstants.dropper_mid);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    arm.setTargetPosition(ArmConstants.arm_pos_high);
                    extension.setTargetPosition(ArmConstants.extension_pos_high);
                })
                .build();

        //ball drop
        TrajectorySequence DropBall = drive.trajectorySequenceBuilder(shippingHub)
                .addTemporalMarker(()->{
                    turret.setTargetPosition(-200);
                }).waitSeconds(1)
                .addTemporalMarker(() -> {
                    servo1.setPosition(ArmConstants.dropper_high);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    carousel.setPower(ArmConstants.out_power);
                }).waitSeconds(1.5)
                .addTemporalMarker(()->{
                    carousel.setPower(0);
                })
                .addTemporalMarker(()->{
                    servo1.setPosition(ArmConstants.dropper_mid);
                })
                .addTemporalMarker(()->{
                    turret.setTargetPosition(0);
                })
                .build();

        //pickup ball
        TrajectorySequence pickBall = drive.trajectorySequenceBuilder(start)
                .addTemporalMarker(()->{
                    servo1.setPosition(ArmConstants.dropper_toPick);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    arm.setTargetPosition(ArmConstants.arm_pos_toPick);
                    extension.setTargetPosition(ArmConstants.extension_pos_toPick);
                }).waitSeconds(1)
                .addTemporalMarker(()->{
                    extension.setTargetPosition(ArmConstants.extension_pos_low);
                    arm.setTargetPosition(ArmConstants.arm_pos_low);
                }).waitSeconds(1)
                .addTemporalMarker(()->{
                    servo1.setPosition(ArmConstants.dropper_low);
                })
                .addTemporalMarker(()->{
                    carousel.setPower(ArmConstants.out_power);
                }).waitSeconds(1.5)
                .addTemporalMarker(()->{
                    carousel.setPower(0);
                })
                .addTemporalMarker(()->{
                    servo1.setPosition(ArmConstants.dropper_mid);
                })
                .build();

//        Pose2d start = new Pose2d(24+startOffset,-66,Math.toRadians(0));
//        Pose2d shippingHub = new Pose2d(-16,-36-8,Math.toRadians(0));

//       ####################     warehouse -> red shipping hub    #############################
        //traverseal (warehouse -> red shipping hub)
        TrajectorySequence BackTraverse = drive.trajectorySequenceBuilder(start)
                .back(startOffset)
                .splineTo(new Vector2d(-16,-36-8),Math.toRadians(180))
                .addTemporalMarker(()->{
                    drive.followTrajectorySequence(DropBall);
                }).waitSeconds(2)
                .build();

//      ####################   red alliance shipping hub -> warehouse   ###########################
        //traversal (red alliance shipping hub -> warehouse)
        TrajectorySequence ForwardTraverse = drive.trajectorySequenceBuilder(shippingHub)
//                .strafeRight(0.1)
//                .splineToConstantHeading(new Vector2d(24-12,-66),Math.toRadians(0))
//                .forward(startOffset+12)
                .strafeRight(24)
                .forward(40)
                .forward(10) //8
                .addTemporalMarker(()->{
                    drive.followTrajectorySequence(pickBall);
                })
                .build();

//        ##############################  code starts ##################################################
        drive.followTrajectorySequence(PickToTop);

        waitForStart();
        runtime.reset();

        while(opModeIsActive())
        {
            drive.followTrajectorySequence(BackTraverse);
            drive.followTrajectorySequence(ForwardTraverse);
            drive.followTrajectorySequence(PickToTop);
        }
        if (isStopRequested())  return;
    }
}
