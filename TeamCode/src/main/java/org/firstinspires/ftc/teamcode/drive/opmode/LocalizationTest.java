package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    DcMotorEx arm,extension;
    int pos = 0;
    int sleepTime = 300;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-24-24-12,24+12,0));
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Arm config
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.armpidcoeffs);
//        arm.setPositionPIDFCoefficients(DriveConstants.armpositionpidcoff);

        // Extension config
        extension = hardwareMap.get(DcMotorEx.class, "extension");
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        extension.setTargetPosition(0);
        extension.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension.setPower(1);


        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            if(gamepad1.dpad_up)
            {
                arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.armpidcoeffs);

                if(pos == 0)
                {
                    extension.setTargetPosition(-200);
                    arm.setTargetPosition(200);
                    pos = 1;
                    sleep(sleepTime);
                    //-200
                }
                else if(pos == 1)
                {
                    extension.setTargetPosition(-300);
                    arm.setTargetPosition(400);
                    //-300
                    pos = 2;
                    sleep(sleepTime);
                }
                else if(pos == 2)
                {
                    extension.setTargetPosition(-450); //-350
                    arm.setTargetPosition(650);
                    pos = 3;
                    sleep(sleepTime);
                    //-200
                }
                else if(pos == 3)
                {
                    extension.setTargetPosition(-550);
                    arm.setTargetPosition(950);
                    pos = 4;
                    sleep(sleepTime);
                    //-100
                }
            }
            else if(gamepad1.dpad_down)
            {
                arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.armdownpidcoeffs);

                if(pos == 4)
                {
                    extension.setTargetPosition(-450);
                    arm.setTargetPosition(650);
                    pos = 3;
                    sleep(sleepTime);
                    //-200
                }
                else if(pos == 3)
                {
                    extension.setTargetPosition(-300);
                    arm.setTargetPosition(400);
                    pos = 2;
                    sleep(sleepTime);
                    //-300
                }
                else if(pos == 2)
                {
                    extension.setTargetPosition(-200);
                    arm.setTargetPosition(200);
                    pos = 1;
                    sleep(sleepTime);
                    //-200
                }
                else if(pos == 1)
                {
                    arm.setTargetPosition(50);
                    extension.setTargetPosition(-65);
                    pos = 0;
                    sleep(sleepTime);
                }
            }
            else if(gamepad1.x)
            {
                extension.setTargetPosition(extension.getCurrentPosition() + DriveConstants.extensionInc);
                extension.setVelocity(DriveConstants.extensionVel);
            }
            else if(gamepad1.a)
            {
                extension.setTargetPosition(extension.getCurrentPosition() - DriveConstants.extensionInc);
                extension.setVelocity(DriveConstants.extensionVel);
            }

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
