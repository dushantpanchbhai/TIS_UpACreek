package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name="Joystick Control", group="Linear Opmode")
//@Disabled
//  use this for joystick control

public class JoystickControl extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Arm and Extension and carousel
    private DcMotorEx arm, extension, carousel, turret;

    Servo servo1,servo2;
    int pos = 1;

    public static PIDFCoefficients turret_pid = new PIDFCoefficients(10,0,0,10);

    public static boolean pidOn = true;

    public static double factor = 0.8;

    @Override
    public void runOpMode() {

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

        waitForStart();
        runtime.reset();

        // move arm to top
        TrajectorySequence PickToTop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(() -> {
                    servo1.setPosition(ArmConstants.dropper_mid);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    arm.setTargetPosition(ArmConstants.arm_pos_high);
                    extension.setTargetPosition(ArmConstants.extension_pos_high);
                })
                .build();

        // used to drop ball from top
        TrajectorySequence DropBall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(() -> {
                    servo1.setPosition(ArmConstants.dropper_high);
                }).waitSeconds(1)
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

        // used to drop ball from mid and high pos
        TrajectorySequence DropBallMidLow = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(()->{
                    servo1.setPosition(ArmConstants.dropper_drop_mid_low);
                }).waitSeconds(1)
                .addTemporalMarker(()->{
                    carousel.setPower(ArmConstants.out_power);
                }).waitSeconds(1.5)
                .build();

        // move arm to lowest position
        TrajectorySequence pickBall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(()->{
                    servo1.setPosition(ArmConstants.dropper_toPick);
                }).waitSeconds(1)
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
                .build();

        //move arm to mid position
        TrajectorySequence holdBall = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(()->{
                    servo1.setPosition(ArmConstants.dropper_mid);
                }).waitSeconds(1)
                .addTemporalMarker(()->{
                    arm.setTargetPosition(ArmConstants.arm_pos_mid);
                    extension.setTargetPosition(ArmConstants.extension_pos_mid);
                })
                .build();

        //move arm to low drop position
        TrajectorySequence idlePick = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(()->{
                    servo1.setPosition(ArmConstants.dropper_toPick);
                }).waitSeconds(1)
                .addTemporalMarker(()->{
                    arm.setTargetPosition(ArmConstants.arm_pos_toPick);
                    extension.setTargetPosition(ArmConstants.extension_pos_toPick);
                })
                .build();


        while(opModeIsActive())
        {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();

            //arm
            if(gamepad1.a)
            {
                //arm pickup pos
                arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,DriveConstants.armdownpidcoeffs);
                drive.followTrajectorySequence(pickBall);
                pos = 1;
            }
            else if(gamepad1.x)
            {
                //arm mid drop pos
                if(pos == 1)
                {
                    arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,DriveConstants.armpidcoeffs);
                }
                else if(pos == 3)
                {
                    arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,DriveConstants.armdownpidcoeffs);
                }
                drive.followTrajectorySequence(holdBall);
                pos = 2;
            }
            else if(gamepad1.y)
            {
                //arm highest pos
                pos = 3;
                arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.armpidcoeffs);
                drive.followTrajectorySequence(PickToTop);
            }
            else if(gamepad1.b)
            {
                //arm low drop position
                if(pos == 1)
                {
                    arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,DriveConstants.armpidcoeffs);
                }
                else if(pos == 3)
                {
                    arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,DriveConstants.armdownpidcoeffs);
                }
                drive.followTrajectorySequence(idlePick);
                pos = 1;
            }

            //manual drop down
            if(gamepad1.dpad_up)
            {
                drive.followTrajectorySequence(DropBall);
            }
            else if(gamepad1.dpad_down)
            {
                drive.followTrajectorySequence(DropBallMidLow);
            }

            //extension manual control
            if(gamepad2.dpad_up)
            {
                extension.setTargetPosition(extension.getCurrentPosition() + DriveConstants.extensionInc);
            }
            else if(gamepad2.dpad_down)
            {
                extension.setTargetPosition(extension.getCurrentPosition() - DriveConstants.extensionInc);
            }

            //turret  control
            if(gamepad1.dpad_right)
            {
                if(turret.getCurrentPosition() < 270)
                {
                    if(pos == 1)
                    {
                        turret.setTargetPosition(turret.getCurrentPosition() + ArmConstants.turret_slow_inc);
                    }
                    else
                    {
                        turret.setTargetPosition(turret.getCurrentPosition() + ArmConstants.turret_fast_inc);
                    }
                }
                else
                {
                    turret.setTargetPosition(270);
                }
            }
            else if(gamepad1.dpad_left)
            {
                if(turret.getCurrentPosition() > -270)
                {
                    if(pos == 1)
                    {
                        turret.setTargetPosition(turret.getCurrentPosition() - ArmConstants.turret_slow_inc);
                    }
                    else
                    {
                        turret.setTargetPosition(turret.getCurrentPosition() - ArmConstants.turret_fast_inc);
                    }
                }
                else
                {
                    turret.setTargetPosition(-270);
                }
            }

            //intaker
            if(gamepad1.left_bumper)
            {
                carousel.setPower(-1);
            }
            else if(gamepad1.right_bumper)
            {
                carousel.setPower(1);
            }
            else
            {
                carousel.setPower(0);
            }

            //rotator
            if(gamepad2.b)
            {
                servo2.setPosition(1);
            }
            else
            {
                servo2.setPosition(0.5);
            }

            //servo1 manual
            if(gamepad1.right_trigger > 0)
            {
                servo1.setPosition(servo1.getPosition() - ArmConstants.servo_inc);
                sleep(10);
            }
            else if(gamepad1.left_trigger > 0)
            {
                servo1.setPosition(servo1.getPosition() + ArmConstants.servo_inc);
                sleep(10);
            }

            telemetry.addData("pos is ",pos);
            telemetry.addData("extension motor current",extension.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("arm postition",arm.getCurrentPosition());
            telemetry.addData("extension position",extension.getCurrentPosition());
            telemetry.addData("turret position",turret.getCurrentPosition());
            telemetry.addData("left_stick", -gamepad1.left_stick_y);
            telemetry.addData("right_stick",-gamepad1.right_stick_x);
            telemetry.update();
        }
        if (isStopRequested())  return;
    }
    public void moveArm(int armTarget){
        arm.setTargetPosition(armTarget);
        arm.setPower(0.6);
    }
}
