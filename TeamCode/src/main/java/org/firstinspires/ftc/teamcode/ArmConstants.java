package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ArmConstants {
    public static double vel_p=0.0,vel_i=0.0,vel_d=0.0,vel_f=0.0;
    public static int arm_velocity = 10;
    public static int arm_pos_low = 50; //50
    public static int arm_pos_mid = 380; //500
    public static int arm_pos_high = 950; //
    public static int arm_pos_toPick = 150;

    public static int extension_pos_low = 30; // 30
    public static int extension_pos_mid = -200; //-550
    public static int extension_pos_high = -420; //-420
    public static int extension_pos_toPick = 0;

    public static double dropper_low = 0.2;
    public static double dropper_mid =0;
    public static double dropper_high = 1;
    public static double dropper_toPick = 0;
    public static double dropper_drop_mid_low = 0.075;

    public static int extension_inc = 20;

    public static int turret_slow_inc = 20;
    public static int turret_fast_inc = 40;

    public static double servo_inc = 0.01;

    public static double in_power = 1;
    public static double out_power = -0.7;
}
