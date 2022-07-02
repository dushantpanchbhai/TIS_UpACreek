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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name="Joystick control", group="Linear Opmode")
//@Disabled
public class JoystickControl extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Arm and Extension and carousel
    private DcMotorEx arm, extension, carousel, turret;

    Servo servo1,servo2;
    static final double INCREMENT   = 0.1;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    int ARM_HIGHEST_POS = 1000;
    int ARM_LOWEST_POS;

    int TURRET_MAX_RIGHT;
    int TURRET_MAX_LEFT;

    //    int
    double position2 = 0;
    boolean carouselActive = false;

//    public static PIDFCoefficients armpidcoeffs = new PIDFCoefficients(5, 0, 0, 0);
//    public PIDFCoefficients armpidcoeffs = DriveConstants.armpidcoeffs;
//    public int armPositionPidCoefficient= DriveConstants.armpositionpidcoff;

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

        // Extension config
        extension = hardwareMap.get(DcMotorEx.class, "extension");
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Carousel
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        //turret
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Get arm and extension ready
        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        extension.setTargetPosition(0);
        extension.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension.setPower(1);

        turret.setTargetPosition(0);
        turret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turret.setPower(1);

        position2 = servo2.getPosition();

        waitForStart();
        runtime.reset();

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
                arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.armdownpidcoeffs);
                if(arm.getCurrentPosition() > 0)
                {
                    arm.setTargetPosition(arm.getCurrentPosition() - 80);
                }
            }
            else if(gamepad1.x)
            {
                arm.setPower(1);
                arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.armpidcoeffs);
                if(arm.getCurrentPosition() < 1000)
                {
                    arm.setTargetPosition(arm.getCurrentPosition() + 80);
                }
                else if(arm.getCurrentPosition() > 1000)
                {
                    arm.setTargetPosition(1000);
                }
            }
            else if(gamepad1.y)
            {
                extension.setTargetPosition(extension.getCurrentPosition() + 20);
            }
            else if(gamepad1.b)
            {
                extension.setTargetPosition(extension.getCurrentPosition() - 20);
            }
            else if(gamepad1.dpad_up)
            {
                carousel.setPower(1);
            }
            else if(gamepad1.dpad_right)
            {
                carousel.setPower(0);
            }
            else if(gamepad1.dpad_down)
            {
                carousel.setPower(-1);
            }
            else if(gamepad1.right_bumper)
            {
                turret.setTargetPosition(turret.getCurrentPosition() + 20);
            }
            else if(gamepad1.left_bumper)
            {
                turret.setTargetPosition(turret.getCurrentPosition() - 20);
            }
            else if(gamepad1.left_trigger > 0)
            {
                position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                }
            }
            else if(gamepad1.right_trigger > 0)
            {
                position -= INCREMENT ;
                if (position <= MIN_POS ) {
                    position = MIN_POS;
                }
            }


            if(gamepad1.dpad_left)
            {
                servo2.setPosition(1);
                telemetry.addData("servo status",servo2.getPosition());
                telemetry.addData("servo direction",servo2.getDirection());
            }
            else
            {
                servo2.setPosition(0.5);
            }

            servo1.setPosition(position);
            sleep(CYCLE_MS);
            idle();

            telemetry.addData("arm postition",arm.getCurrentPosition());
            telemetry.addData("extension position",extension.getCurrentPosition());
            telemetry.addData("carousel power",carousel.isMotorEnabled());
            telemetry.addData("turret position",turret.getCurrentPosition());
            telemetry.addData("servo status",servo2.getPosition());
            telemetry.addData("servo direction",servo2.getDirection());
            telemetry.update();
        }
        if (isStopRequested())  return;
    }
}
