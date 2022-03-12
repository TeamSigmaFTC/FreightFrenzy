package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
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
import java.util.PriorityQueue;

// Credits to team 7303 RoboAvatars and 3954 Pink to the Future, improved by Team 18272 Sigma

public class ContourPipeline extends OpenCvPipeline {
    Scalar HOT_PINK = new Scalar(196, 23, 112);

    // Red                                               Y      Cr     Cb    (Do not change Y)
//    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 170.0, 0.0);
//    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 150.0);

    // Blue                                               Y      Cr     Cb    (Do not change Y)
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 150);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 160.0, 255.0);


    // Green                                             Y      Cr     Cb
//     public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 0.0, 0.0);
//     public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);

    //Volatile bc accessed by opmode without sync
    public volatile boolean error = false;
    public volatile Exception debug;

    private double borderLeftX = 0.0;   //fraction of pixels from the left side of the cam to skip
    private double borderRightX = 0.0;   //fraction of pixels from the right of the cam to skip
    private double borderTopY = 0.0;   //fraction of pixels from the top of the cam to skip
    private double borderBottomY = 0.0;   //fraction of pixels from the bottom of the cam to skip

    private int CAMERA_WIDTH;
    private int CAMERA_HEIGHT;

    private Mat mat = new Mat();
    private Mat processed = new Mat();

    private PriorityQueue<Rect> rects = new PriorityQueue<Rect>((a, b) -> (a.area() < b.area() ? -1 : 1));

    private final Object sync = new Object();

    public ContourPipeline(double borderLeftX, double borderRightX, double borderTopY, double borderBottomY) {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
    }

    public void configureScalarLower(double y, double cr, double cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarUpper(double y, double cr, double cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarLower(int y, int cr, int cb) {
        scalarLowerYCrCb = new Scalar(y, cr, cb);
    }

    public void configureScalarUpper(int y, int cr, int cb) {
        scalarUpperYCrCb = new Scalar(y, cr, cb);
    }

    @Override
    public Mat processFrame(Mat input) {
        CAMERA_WIDTH = input.width();
        CAMERA_HEIGHT = input.height();
        try {
            // Process Image
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(mat, scalarLowerYCrCb, scalarUpperYCrCb, processed);
            // Core.bitwise_and(input, input, output, processed);

            // Remove Noise
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());
            // GaussianBlur
            Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);
            // Find Contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw Contours
            Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

            //lock this up to prevent errors when outside threads access the max rect property.
            synchronized (sync) {
                rects.clear();
                // Loop Through Contours
                for (MatOfPoint contour : contours) {
                    Point[] contourArray = contour.toArray();

                    // Bound Rectangle if Contour is Large Enough
                    if (contourArray.length >= 15) {
                        MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                        Rect rect = Imgproc.boundingRect(areaPoints);

                        // if rectangle is larger than previous cycle or if rectangle is not larger than previous 6 cycles > then replace

                        if (rect.area() > 300
                                && rect.x >= (borderLeftX * CAMERA_WIDTH) && rect.x + rect.width <= CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH)
                                && rect.y >= (borderTopY * CAMERA_HEIGHT) && rect.y + rect.height <= CAMERA_HEIGHT - (borderBottomY * CAMERA_HEIGHT)
                                ) {
                            rects.add(rect);
                            if(rects.size() > 2) {
                                // only keep 2 biggest one, remove the smallest if more than 2
                                rects.remove();
                            }
                        }
                        areaPoints.release();
                    }
                    contour.release();
                }
            }
            // Draw Rectangles
            int index = 0;
            for(Rect rect: rects) {
                Imgproc.rectangle(input, rect, new Scalar(0, 255, 0), 2);
                Imgproc.putText(input, "Area: " + rect.area() + " Midpoint: " + getRectMidpointX(rect) + " , " + getRectMidpointY(rect), new Point(5, CAMERA_HEIGHT - 15 * (index+1)), 0, 0.6, new Scalar(255, 255, 255), 2);
                index++;
            }

            if(rects.isEmpty()) {
                Imgproc.putText(input, "nothing found x " + contours.size(), new Point(5, CAMERA_HEIGHT - 5), 0, 0.6, new Scalar(255, 255, 255), 2);
            }
            // Draw Borders
            Imgproc.rectangle(input, new Rect(
                    (int) (borderLeftX * CAMERA_WIDTH),
                    (int) (borderTopY * CAMERA_HEIGHT),
                    (int) (CAMERA_WIDTH - (borderRightX * CAMERA_WIDTH) - (borderLeftX * CAMERA_HEIGHT)),
                    (int) (CAMERA_HEIGHT - (borderBottomY * CAMERA_WIDTH) - (borderTopY * CAMERA_HEIGHT))
            ), HOT_PINK, 2);

        } catch (Exception e) {
            debug = e;
            error = true;
        }
        return input;
    }
    /*
    Synchronize these operations as the user code could be incorrect otherwise, i.e a property is read
    while the same rectangle is being processed in the pipeline, leading to some values being not
    synced.
     */

    public PriorityQueue<Rect> getRects() {
        synchronized (sync) {
            return new PriorityQueue(rects);
        }
    }

    public static double getRectMidpointX(Rect r) {
        return r.x + (r.width / 2.0);
    }

    public static double getRectMidpointY(Rect r) {
        return r.y + (r.height / 2.0);
    }

    public double getCameraWidth() {
        synchronized (sync) {
            return CAMERA_WIDTH;
        }
    }

    //TODO: REMOVE
    public double getRectArea() { return 0; }
    public double getRectMidpointX() { return 0; }
}