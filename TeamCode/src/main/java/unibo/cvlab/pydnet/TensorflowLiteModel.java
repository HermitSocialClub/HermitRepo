package unibo.cvlab.pydnet;

import android.graphics.Bitmap;

import org.opencv.core.Mat;
import org.tensorflow.lite.Interpreter;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.HashMap;
import java.util.Map;

public class TensorflowLiteModel extends Model {

    protected Interpreter tfLite;
    private ByteBuffer outputByteBuffer;
    private ByteBuffer inputByteBuffer;
    private int[] intInputPixels;
    private boolean isPrepared = false;

    public TensorflowLiteModel(String name, File modelFile) {
        super(name);

        Interpreter.Options tfliteOptions = new Interpreter.Options();
        // <Aditya> also make sure when you init pydnet with tflite to use xnnpack
        tfliteOptions.setUseXNNPACK(true);
        this.tfLite = new Interpreter(modelFile, tfliteOptions);
    }

    @Override
    public void prepare(Utils.Resolution resolution) {
        outputByteBuffer = ByteBuffer.allocateDirect(resolution.getHeight() * resolution.getWidth() * 4);
        inputByteBuffer = ByteBuffer.allocateDirect(3 * resolution.getHeight() * resolution.getWidth() * 4);
        intInputPixels = new int[resolution.getWidth() * resolution.getHeight()];
        inputByteBuffer.order(ByteOrder.nativeOrder());
        outputByteBuffer.order(ByteOrder.nativeOrder());
        isPrepared = true;
    }

    public FloatBuffer doInference(Mat input, Utils.Scale scale) {
        if (!isPrepared) {
            throw new RuntimeException("Model is not prepared.");
        }
        inputByteBuffer.rewind();
        outputByteBuffer.rewind();

        fillInputByteBufferWithBitmap(input);
        inputByteBuffer.rewind();
        Object[] inputArray = new Object[tfLite.getInputTensorCount()];
        inputArray[tfLite.getInputIndex(getInputNode("image"))] = inputByteBuffer;

        Map<Integer, Object> outputMap = new HashMap<>();
        outputMap.put(tfLite.getOutputIndex(outputNodes.get(scale)), outputByteBuffer);

        this.tfLite.runForMultipleInputsOutputs(inputArray, outputMap);

        outputByteBuffer.rewind();
        return outputByteBuffer.asFloatBuffer();
    }

    // Helper methods

    private void fillInputByteBufferWithBitmap(Mat bitmap) {
        bitmap.get(0, 0, intInputPixels);
        // Convert the image to floating point.
        int pixel = 0;
        for (int i = 0; i < bitmap.width(); ++i) {
            for (int j = 0; j < bitmap.height(); ++j) {
                final int val = intInputPixels[pixel++];
                inputByteBuffer.putFloat((((val >> 16) & 0xFF)) / (float) 255.);
                inputByteBuffer.putFloat((((val >> 8) & 0xFF)) / (float) 255.);
                inputByteBuffer.putFloat((((val) & 0xFF)) / (float) 255.);
            }
        }
    }
}
