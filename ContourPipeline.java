package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

// Credits to team 7303 RoboAvatars, adjusted by team 3954 Pink to the Future

public class ContourPipeline extends OpenCvPipeline
{
    Scalar PINK = new Scalar(196, 23, 112);

    public static Scalar scalarLowerRGB = new Scalar(100, 100, 100);
    public static Scalar scalarUpperRGB = new Scalar(255, 200, 200);

    public boolean error = false;
    public Exception debug;

    private int borderLeftX   = 0;   //amount of pixels from the left side of the cam to skip
    private int borderRightX  = 0;   //amount of pixels from the right of the cam to skip
    private int borderTopY    = 0;   //amount of pixels from the top of the cam to skip
    private int borderBottomY = 0;   //amount of pixels from the bottom of the cam to skip

    private int CAMERA_WIDTH = 640;
    private int CAMERA_HEIGHT = 360;

    private int loopcounter = 0;
    private int ploopcounter = 0;

    private Mat mat = new Mat();
    private Mat processed = new Mat();

    private Rect maxRect = new Rect();

    private double maxArea = 0;

    public void ConfigurePipeline(int borderLeftX, int borderRightX, int borderTopY, int borderBottomY, int CAMERA_WIDTH, int CAMERA_HEIGHT)
    {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
        this.CAMERA_WIDTH = CAMERA_WIDTH;
        this.CAMERA_HEIGHT = CAMERA_HEIGHT;
    }

    public void ConfigureScalarLower(double Red, double Green, double Blue) { scalarLowerRGB = new Scalar(Red, Green, Blue); }
    public void ConfigureScalarUpper(double Red, double Green, double Blue) { scalarUpperRGB = new Scalar(Red, Green, Blue); }
    public void ConfigureScalarLower(int Red, int Green, int Blue) { scalarLowerRGB = new Scalar(Red, Green, Blue); }
    public void ConfigureScalarUpper(int Red, int Green, int Blue) { scalarUpperRGB = new Scalar(Red, Green, Blue); }

    public double downScaleFactor = 0.6;
    private Mat workingMat = new Mat();
    private Mat blurredMat = new Mat();
    private Mat maskRed = new Mat();
    private Mat maskBlue = new Mat();
    private Mat hierarchy = new Mat();


    private Size newSize = new Size();
    @Override
    public Mat processFrame(Mat input)
    {

        Size initSize = input.size();
        newSize = new Size(initSize.width * downScaleFactor, initSize.height * downScaleFactor);
        input.copyTo(workingMat);

        Imgproc.resize(workingMat, workingMat, newSize);

        Mat redConvert = workingMat.clone();
        Mat blueConvert = workingMat.clone();

        List<MatOfPoint> contoursRed = new ArrayList<>();

        Imgproc.findContours(getRedMask(redConvert), contoursRed, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(workingMat, contoursRed, -1, new Scalar(230,70,70));
//
//
//
//        // Process Image
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2BGR);
//        Core.inRange(mat, new Scalar(scalarLowerRGB.val[2], scalarLowerRGB.val[1], scalarLowerRGB.val[0]), new Scalar(scalarUpperRGB.val[2], scalarUpperRGB.val[1], scalarUpperRGB.val[0]), processed);
//
//        // Remove Noise
//        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
//        Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
//
//        // Find Contours
//        List<MatOfPoint> contours = new ArrayList<>();
//        Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        Imgproc.drawContours(input, contours, -1, new Scalar(255,0,0));
//
//        // Loop Through Contours
//        for (MatOfPoint contour : contours) {
//            Point[] contourArray = contour.toArray();
//
//            // Bound Rectangle if Contour is Large Enough
//            if (contourArray.length >= 5) {
//                MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
//                Rect rect = Imgproc.boundingRect(areaPoints);
//
//                // if rectangle is larger than previous cycle or if rectangle is not larger than previous 6 cycles > then replace
//                if (rect.area() > maxArea
//                        && rect.x > borderLeftX && rect.x + rect.width < CAMERA_WIDTH - borderRightX
//                        && rect.y > borderTopY && rect.y + rect.height < CAMERA_HEIGHT - borderBottomY
//                        || loopcounter-ploopcounter > 6) {
//                    maxArea = rect.area();
//                    maxRect = rect;
//                    ploopcounter++;
//                    loopcounter = ploopcounter;
//                }
//            }
//        }
//        // Draw Rectangles
//        Imgproc.rectangle(input, maxRect, new Scalar(0,255,0),1);
//        Imgproc.rectangle(input, new Rect(borderLeftX, borderTopY, CAMERA_WIDTH - borderRightX, CAMERA_HEIGHT - borderBottomY), PINK, 2);
        loopcounter++;

        return workingMat;
    }
    private Mat getRedMask(Mat input){
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2Lab);
        Imgproc.GaussianBlur(input, input, new Size(3,3), 0);
        List<Mat> channels = new ArrayList<>();
        Core.split(input, channels);
        Imgproc.threshold(channels.get(1), maskRed, 164.0, 255, Imgproc.THRESH_BINARY);

        return maskRed;
    }
    private Mat getBlueMask(Mat input){
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YUV);
        Imgproc.GaussianBlur(input, input, new Size(3,3), 0);
        List<Mat> channels = new ArrayList<>();
        Core.split(input, channels);
        Imgproc.threshold(channels.get(1), maskBlue, 145.0, 255, Imgproc.THRESH_BINARY);

        return maskBlue;
    }
    public int getRectHeight(){return maxRect.height;}
    public int getRectWidth(){ return maxRect.width; }
    public int getRectX(){ return maxRect.x; }
    public int getRectY(){ return maxRect.y; }
    public double getRectMidpointX(){ return getRectX() + (getRectWidth()/2.0); }
    public double getRectMidpointY(){ return getRectY() + (getRectHeight()/2.0); }
    public double getAspectRatio(){ return getRectArea()/(CAMERA_HEIGHT*CAMERA_WIDTH); }
    public double getRectArea(){ return maxRect.area(); }
}