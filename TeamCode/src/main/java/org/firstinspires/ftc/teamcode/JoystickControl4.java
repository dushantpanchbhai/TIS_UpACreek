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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name="Joystick control 4", group="Linear Opmode")
//@Disabled
public class JoystickControl4 extends LinearOpMode {

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

    int EXTENSTION_MIN_POS = -100;

    //    int
    double position2 = 0;
    boolean carouselActive = false;

    double y = (DriveConstants.linearM*DriveConstants.armInc) + DriveConstants.linearC;

    int armPosition;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");

        //mechanum drive
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        armPosition = arm.getCurrentPosition();

        waitForStart();
        runtime.reset();

        while(opModeIsActive())
        {
            //arm
            if(gamepad1.dpad_down)
            {
                arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.armdownpidcoeffs);
                if(arm.getCurrentPosition() > 500)
                {
                    arm.setTargetPosition(arm.getCurrentPosition() - DriveConstants.armDec);
                }
//                else if(arm.getCurrentPosition() < 500)
                else
                {
                    arm.setTargetPosition(500);
                }

                armPosition -= DriveConstants.armDec;
//                extension.setTargetPosition(extension.getCurrentPosition() - (int) DriveConstants.y);
            }
            else if(gamepad1.dpad_up)
            {
                arm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, DriveConstants.armpidcoeffs);
                if(arm.getCurrentPosition() < 1000)
                {
                    arm.setTargetPosition(arm.getCurrentPosition() + DriveConstants.armInc);
                    extension.setTargetPosition(extension.getCurrentPosition() + (int) DriveConstants.y);
                    arm.setPower(1);
                    armPosition+=DriveConstants.armInc;
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
            //
            else if(gamepad1.dpad_right)
            {
                turret.setTargetPosition(turret.getCurrentPosition() + 20);
            }
            else if(gamepad1.dpad_left)
            {
                turret.setTargetPosition(turret.getCurrentPosition() - 20);
            }

            if(gamepad1.b)
            {
                servo1.setPosition(1);
            }
            else if(gamepad1.y)
            {
                servo1.setPosition(0);
            }
            else
            {
                servo1.setPosition(0.5);
            }

            if(gamepad1.left_trigger > 0)
            {
                servo2.setPosition(1);
            }
            else if(gamepad1.right_trigger > 0)
            {
                servo2.setPosition(0);
            }
            else
            {
                servo2.setPosition(0.5);
            }

            //intaker
            if(gamepad1.left_bumper)
            {
                carousel.setPower(1);
            }
            else if(gamepad1.right_bumper)
            {
                carousel.setPower(-1);
            }
            else
            {
                carousel.setPower(0);
            }

            telemetry.addData("extension motor current",extension.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("extension motor speed",extension.getVelocity());
            telemetry.addData("arm postition",arm.getCurrentPosition());
            telemetry.addData("extension position",extension.getCurrentPosition());
            telemetry.addData("carousel power",carousel.isMotorEnabled());
            telemetry.addData("turret position",turret.getCurrentPosition());
            telemetry.addData("servo status",servo2.getPosition());
            telemetry.addData("servo direction",servo2.getDirection());
            telemetry.addData("value of y is : ",y);
            telemetry.addData("actual arm position",armPosition);
            telemetry.update();
        }
        if (isStopRequested())  return;
    }
    public void moveArm(int armTarget){
        arm.setTargetPosition(armTarget);
        arm.setPower(0.6);
    }
}
