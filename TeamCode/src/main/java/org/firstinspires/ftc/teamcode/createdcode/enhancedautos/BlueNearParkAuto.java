package org.firstinspires.ftc.teamcode.createdcode.enhancedautos;

import static org.firstinspires.ftc.teamcode.createdcode.configs.AutoConfig.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(group = "Blue")
@Config
public class BlueNearParkAuto extends EnhancedAutoMode {
    public static double START_POS_X = -32;
    public static double START_POS_Y = 66;
    public static double START_POS_ANGLE = 270;
    public static int START_METHOD = 20;

    public static int BLOCK_DROP_POINT = -30;

    public static int SHIPPING_ELEMENT_CENTER_HEIGHT = 135;
    public static int SHIPPING_ELEMENT_POS_1_X = 30;
    public static int SHIPPING_ELEMENT_POS_2_X = 165;
    public static int SHIPPING_ELEMENT_POS_3_X = 285;


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    public static double[] xCoordArr = {};
    private static double[] yCoordArr = {};
    private static double[] angleArr = {};
    private static int[] methodIdArr = {};

    public static double[] xCoordArrCase1 = {START_POS_X, -48, -48, BLOCK_DROP_POINT, -48, BLUE_CAROUSEL_X, BLUE_STORAGE_X};
    private static final double[] yCoordArrCase1 = {START_POS_Y, 48, 24, 24, 24, BLUE_CAROUSEL_Y, BLUE_STORAGE_Y};
    private static final double[] angleArrCase1 = {START_POS_ANGLE, 180, 90, 0, 0, BLUE_CAROUSEL_ANGLE, BLUE_STORAGE_ANGLE};
    private static final int[] methodIdArrCase1 = {START_METHOD, 13, 0, 22, 10, 32, 0};

    public static double[] xCoordArrCase2 = {START_POS_X, -48, -48, BLOCK_DROP_POINT, -48, BLUE_CAROUSEL_X, BLUE_STORAGE_X};
    private static final double[] yCoordArrCase2 = {START_POS_Y, 48, 24, 24, 24, BLUE_CAROUSEL_Y, BLUE_STORAGE_Y};
    private static final double[] angleArrCase2 = {START_POS_ANGLE, 180, 90, 0, 0, BLUE_CAROUSEL_ANGLE, BLUE_STORAGE_ANGLE};
    private static final int[] methodIdArrCase2 = {START_METHOD, 12, 0, 22, 10, 32, 0};

    public static double[] xCoordArrCase3 = {START_POS_X, BLOCK_DROP_POINT, BLOCK_DROP_POINT, -56, BLUE_CAROUSEL_X, BLUE_STORAGE_X};
    private static final double[] yCoordArrCase3 = {START_POS_Y, 48, 24, 24, BLUE_CAROUSEL_Y, BLUE_STORAGE_Y};
    private static final double[] angleArrCase3 = {START_POS_ANGLE, 135, 0, 0, BLUE_CAROUSEL_ANGLE, BLUE_STORAGE_ANGLE};
    private static final int[] methodIdArrCase3 = {START_METHOD, 11, 22, 10, 32, 0};

    @Override
    public void runOpMode() {
        int pos = getShippingElementPos(SHIPPING_ELEMENT_CENTER_HEIGHT, SHIPPING_ELEMENT_POS_1_X, SHIPPING_ELEMENT_POS_2_X, SHIPPING_ELEMENT_POS_3_X);
        dashboardTelemetry.addData("Position Detected is", pos);
        dashboardTelemetry.update();

        if (pos == 1) {
            xCoordArr = xCoordArrCase1;
            yCoordArr = yCoordArrCase1;
            angleArr = angleArrCase1;
            methodIdArr = methodIdArrCase1;
        } else if (pos == 2) {
            xCoordArr = xCoordArrCase2;
            yCoordArr = yCoordArrCase2;
            angleArr = angleArrCase2;
            methodIdArr = methodIdArrCase2;
        } else if (pos == 3) {
            xCoordArr = xCoordArrCase3;
            yCoordArr = yCoordArrCase3;
            angleArr = angleArrCase3;
            methodIdArr = methodIdArrCase3;
        }


        initThings();

        setXCoords(xCoordArr);
        setYCoords(yCoordArr);
        setAngles(angleArr);
        setMethodIDS(methodIdArr);

        try {
            makeActionObjects();
        } catch (Exception e) {
            e.printStackTrace();
        }

        waitForStart();

        run();
    }

}
