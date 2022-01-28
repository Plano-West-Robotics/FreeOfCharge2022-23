package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Vector;

@Config
public class AutoConstants {

    public static double CAROUSEL_POWER = 0.7;
    public static int CAROUSEL_WAIT = 3500;
    public static int ARM_BACK_DIST = 400;
    public static int GRAB_TIME = 1000;
    public static double GRAB_POS = 0;
    public static double OPENED_POS = 0.4;

    //blue values
        //blue1
        public static int BLUE1_START_X = -30;
        public static int BLUE1_START_Y = 65;

        //blue movements
        public static int BLUE_CAROUSEL_X = -60;
        public static int BLUE_CAROUSEL_Y = 50;
        public static int BLUE_START_FORWARD = 40;
        public static int BLUE_FINAL_STRAFE = 14;

    //red values
        public static int RED1_START_X = -30;
        public static int RED1_START_Y = -65;

        //blue movements
        public static int RED_CAROUSEL_X = -67;
        public static int RED_CAROUSEL_Y = -57;
        public static int RED_START_FORWARD = 40;
        public static int RED_FINAL_STRAFE = 14;
}
