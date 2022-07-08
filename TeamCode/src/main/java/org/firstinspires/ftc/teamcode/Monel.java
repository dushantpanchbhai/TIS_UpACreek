package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Monel", group="Final OpModes")
public class Monel extends LinearOpMode{


    public ElapsedTime seconds = new ElapsedTime();
    //####################### TURRET POSITIONS #########################
    int TURRETCENTER = 243;
    int TURRETPICKPOS = 130;

    //##################################################################
    //########################## FLAGS #################################
    int BEGININTAKE = 1;
    int STOPINTAKE = 0;
    int BEGINTRANSFER = 1;
    int STOPTRANSFER = 0;

    int extension_flag = 0;
    int intakeFlag = 0, manualIntakeFlag = 0;
    int transferFlag = 0;
    int droppingFlag = 0;
    int servoExtensionFlag = 0;

    int counterExtension1 = 0, counterExtension2 = 0;

    //##################################################################
    //####################### EXTENSION POSITIONS ######################
    int MAXEXTENSION = -2000;
    int pos = 0;

    //##################################################################
    //############################# HARDWARE ###########################
    DcMotorEx extension_motor1, extension_motor2, turnTable, intake;
    Servo servo_dropper, servo_arm, left_intake, right_intake;
    RevColorSensorV3 left_intake_sensor,  right_intake_sensor;





    public void runOpMode() {
        //##################################################################
        //####################### HARDWARE MAP #############################
        extension_motor1 = hardwareMap.get(DcMotorEx.class, "extension1");
        extension_motor2 = hardwareMap.get(DcMotorEx.class, "extension2");
        servo_arm = hardwareMap.get(Servo.class, "Arm");
        servo_dropper = hardwareMap.get(Servo.class, "Dropper");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turnTable = hardwareMap.get(DcMotorEx.class, "turret");
        left_intake = hardwareMap.get(Servo.class, "leftIntake");
        right_intake = hardwareMap.get(Servo.class, "rightIntake");
        left_intake_sensor = hardwareMap.get(RevColorSensorV3.class, "leftSensor");
        right_intake_sensor = hardwareMap.get(RevColorSensorV3.class, "rightSensor");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        extension_motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extension_motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo_dropper.setPosition(0.3);
        servo_arm.setPosition(0.02);
        left_intake.setPosition(0);
        right_intake.setPosition(1);





        waitForStart();

        extension_motor1.setTargetPosition(0);
        extension_motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension_motor1.setVelocity(1000);
        extension_motor2.setTargetPosition(0);
        extension_motor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension_motor2.setVelocity(1000);
        rotateTo(TURRETPICKPOS);
        seconds.reset();

        while (opModeIsActive() || !isStopRequested()) {
            //-------------------------------------------------------------------------
            //-------------------------------------------------------------------------
            if(gamepad1.start){
                rotateTo(TURRETCENTER);
            }
            else if(gamepad1.back){
                rotateTo(TURRETPICKPOS);
            }
            if(gamepad1.dpad_right){
                intakeFlag = STOPINTAKE;                 //Stop Auto Intake Process
                manualIntakeFlag = BEGININTAKE;          //Start Manual Intake Process
                transferFlag = STOPTRANSFER;             //Stop transfers from taking place
            }
            else if(gamepad1.dpad_up){
                intakeFlag = BEGININTAKE;                //Start the Auto Intake Process
                manualIntakeFlag = STOPINTAKE;           //Stop the Manual Intake Process
                transferFlag = STOPTRANSFER;             //Stop transfers from taking place
            }
            if(gamepad1.dpad_down){
                intakeFlag = STOPINTAKE;
                manualIntakeFlag = STOPINTAKE;
                transferFlag = STOPTRANSFER;
            }



            if((intakeFlag == BEGININTAKE) && !(getProximityReading(right_intake_sensor, 20)) && (manualIntakeFlag == STOPINTAKE)){
                right_intake.setPosition(1);
                setIntakeTo(-1);                   //If Auto Intake is Enabled and sensor readings are over 20, start Intake
            }
            else if((intakeFlag == BEGININTAKE) && (getProximityReading(right_intake_sensor, 20)) && (manualIntakeFlag == STOPINTAKE)){
                setIntakeTo(0);                    //Turn off the Intake since Object has been detected
                transferFlag = BEGINTRANSFER;                  //Ready for Transfer
                intakeFlag = STOPINTAKE;
                right_intake.setPosition(0);       //Rotate the servo to make the intake ready for transfer
                seconds.reset();                   //Reset the timer for transfer counts
            }
            else if((intakeFlag == STOPINTAKE) && (manualIntakeFlag == STOPINTAKE)){
                setIntakeTo(0);                    //Turn the Intake off
            }
            else if((intakeFlag == STOPINTAKE) && (manualIntakeFlag == BEGININTAKE)){
                setIntakeTo(-1);                   //Turn the Intake on
            }

            if((transferFlag == BEGINTRANSFER) && (seconds.milliseconds() > 1000)){
                if(getProximityReading(right_intake_sensor, 20)){
                    setIntakeTo(-1);
                }
            }

            if(transferFlag == BEGINTRANSFER){
                if(!getProximityReading(right_intake_sensor, 20)){
                    setIntakeTo(0);
                }
            }

            telemetry.addData("intakeFlag = ", intakeFlag);
            telemetry.addData("transferFlag = ", transferFlag);
            telemetry.addData("manualIntakeFlag = ", manualIntakeFlag);
            telemetry.addData("Encoder 1 count: ", extension_motor1.getCurrentPosition());
            telemetry.addData("Encoder 2 count: ", extension_motor2.getCurrentPosition());
            telemetry.addData("extensin motor 1 current",extension_motor1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("extensin motor 2 current",extension_motor2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ext 1 position",extension_motor1.getCurrentPosition());
            telemetry.addData("ext 2 position",extension_motor2.getCurrentPosition());
            telemetry.addData("ext 1 position",extension_motor1.getTargetPosition());
            telemetry.addData("ext 2 position",extension_motor2.getTargetPosition());
            telemetry.addData("difference",Math.abs(extension_motor1.getCurrent(CurrentUnit.AMPS) - extension_motor2.getCurrent(CurrentUnit.AMPS)));
            telemetry.update();





            //-------------------------------------------------------------------------
            //-------------------------------------------------------------------------

            if(gamepad1.right_trigger > 0.7){
                pos = pos + 7;
                extendTo(pos);
                sleep(2);
            }
            else
            {
                extendTo(pos);
            }
//            else{
//                pos = pos;
//            }

            if(gamepad1.left_trigger > 0.7){
                pos = pos - 7;
                extendTo(pos);
                sleep(2);
            }
//            else{
//                pos = pos;
//            }

//            extendTo(pos);

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();



            Pose2d poseEstimate = drive.getPoseEstimate();

        }
    }


    public void rotateTo(int pos){
        turnTable.setTargetPosition(pos);
        turnTable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turnTable.setVelocity(500);
    }

    public void extendTo(int pos){
        extension_motor1.setTargetPosition(pos);
        extension_motor2.setTargetPosition(-pos);
        extension_motor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension_motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension_motor1.setVelocity(1800);
        extension_motor2.setVelocity(1800);
    }

    public void setIntakeTo(int power){
        intake.setPower(power);
    }

    public boolean getProximityReading(RevColorSensorV3 sensor, int dist){
        if(sensor.getDistance(DistanceUnit.MM) < dist){
            return true;
        }
        else{
            return false;
        }
    }
}
