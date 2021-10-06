package org.firstinspires.ftc.teamcode.opencv;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class OpenCVPipeline extends OpenCvPipeline {

    public boolean error = false;
    private Mat ret = new Mat();
    private Mat mat = new Mat();
    private Mat mask = new Mat();

    public int getRectX;
    public int getRectY;
    public double getRectArea;
    public int getRectHeight;
    public int getRectWidth;
    public double getRectMidpointX;
    public double getRectMidpointY;
    public double getAspectRatio;

    private int HOR = 0;
    private int HORIZON = 0;
    private int CAMERA_WIDTH = 640;
    private int CAMERA_HEIGHT = 360;

    private int loopcounter = 0;
    private int ploopcounter = 0;

    private Rect maxRect;

    private double maxArea = 0;

    public static Scalar lowerOrange = new Scalar(0.0, 141.0, 0.0);
    public static Scalar upperOrange = new Scalar(255.0, 230.0, 105.0);

    public void ConfigurePipeline(int HORIZON, int HOR, int CAMERA_WIDTH, int CAMERA_HEIGHT){
        this.HORIZON = HORIZON;
        this.HOR = HOR;
        this.CAMERA_WIDTH = CAMERA_WIDTH;
        this.CAMERA_HEIGHT = CAMERA_HEIGHT;
    }
    public void ConfigureScalarLower(double Red, double Green, double Blue){
        lowerOrange = new Scalar(Red,Green,Blue);
    }
    public void ConfigureScalarUpper(double Red, double Green, double Blue){
        upperOrange = new Scalar(Red,Green,Blue);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        ret.release();
        ret = new Mat();

        try {
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
            mask = new Mat(mat.rows(), mat.cols(), CvType.CV_8UC1);
            Core.inRange(mat, lowerOrange, upperOrange, mask);
            Core.bitwise_and(input, input, ret, mask);
            Imgproc.GaussianBlur(mask, mask, new Size(5.0,15.0), 0.00);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

            Imgproc.drawContours(ret, contours, -1, new Scalar(0.0, 255.0, 0.0), 3);

            for (MatOfPoint contour : contours) {
                Point[] contourArray = contour.toArray();
                // if Contour is large enough
                if (contourArray.length >= 5) {
                    MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                    Rect rect = Imgproc.boundingRect(areaPoints);

                    // if rectangle is larger than previous cycle, but delete rectangle if that is true for more than x cycles
                    if (rect.area() > maxArea && rect.x < HOR && rect.y + rect.height > HORIZON || loopcounter-ploopcounter>10) {
                        maxArea = rect.area();
                        maxRect = rect;
                        ploopcounter += 1;
                        loopcounter = ploopcounter;
                    }
                    areaPoints.release();
                }
                contour.release();
            }
            Imgproc.rectangle(ret, maxRect, new Scalar(0.0, 0.0, 255.0), 2);
            double Horizon = HORIZON/100.0*100.0;
            double Hor = HOR/100.0*100.0;
            double Camera_width = CAMERA_WIDTH/100.0*100.0;
            double Camera_height = CAMERA_HEIGHT/100.0*100.0;
            Imgproc.line(ret, new Point(.0,Horizon), new Point(Camera_width, Horizon), new Scalar(255.0,.0, 255.0));
            Imgproc.line(ret, new Point(Hor, .0), new Point(Hor, Camera_height), new Scalar(255.0, .60, 255.0));

            mat.release();
            mask.release();
            hierarchy.release();

            getRectArea = maxRect.area();
            getRectHeight = maxRect.height;
            getRectWidth = maxRect.width;
            getRectX = maxRect.x;
            getRectY = maxRect.y;
            getRectMidpointX = getRectX + (getRectWidth/2.0);
            getRectMidpointY = getRectY + (getRectHeight/2.0);
            getAspectRatio = getRectArea/(CAMERA_HEIGHT*CAMERA_WIDTH);
            loopcounter += 1;

        } catch (Exception e) {
              error = true;
        }
        return ret;
    }
    public double getRectHeight(){
        return getRectHeight;
    }
}
