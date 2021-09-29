package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="OpenCV_Test", group="Tutorials")

public class OpenCV_Tutorial_Freight_Frenzy extends LinearOpMode {

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    //PINK
    public static int R_Lower = 180;
    public static int G_Lower = 0;
    public static int B_Lower = 0;
    public static int R_Upper = 255;
    public static int G_Upper = 150;
    public static int B_Upper = 150;

    @Override
    public void runOpMode()
    {
        //OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourPipeline());
        //Configuration of Pipeline
        myPipeline.ConfigurePipeline(30, 30,30,30,  CAMERA_WIDTH, CAMERA_HEIGHT);
        myPipeline.ConfigureScalarLower(R_Lower,G_Lower,B_Lower);
        myPipeline.ConfigureScalarUpper(R_Upper,G_Upper,B_Upper);
        //Webcam Streaming
        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        //Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            if(myPipeline.error){
                telemetry.addData("Exception: ", myPipeline.debug);
            }

            telemetry.addData("RectArea: ", myPipeline.getRectArea());
            telemetry.update();
            if(myPipeline.getRectArea() > 40) AUTONOMOUS_A();
        }
    }
    public void AUTONOMOUS_A(){
        telemetry.addLine("Autonomous A");
    }
}