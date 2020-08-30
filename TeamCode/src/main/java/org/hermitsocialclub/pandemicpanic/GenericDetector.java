package org.hermitsocialclub.pandemicpanic;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;
import org.opencv.*;

//import kotlin.sequences.DropWhileSequence;

public class GenericDetector extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public GenericDetector () {

    }
    @Override
    public final Mat processFrame (Mat input) {
        input.copyTo(workingMatrix);
        if (workingMatrix.empty()) {

            return input;
        }
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2YCrCb);
        Mat matLeft = workingMatrix.submat(120,150,10,50);
        Mat matCenter = workingMatrix.submat(120,150,80,120);
        Mat matRight = workingMatrix.submat(120,150,150,190);
        return new Mat();
    }
}
