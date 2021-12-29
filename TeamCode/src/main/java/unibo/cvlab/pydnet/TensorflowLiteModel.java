package unibo.cvlab.pydnet;

import org.opencv.core.Mat;
import org.tensorflow.lite.Interpreter;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.HashMap;
import java.util.Map;

public abstract class TensorflowLiteModel extends Model {

    private final boolean quantizeInput;
    protected Interpreter tfLite;
    private ByteBuffer outputByteBuffer;
    private ByteBuffer inputByteBuffer;
    private byte[] byteInputPixels;
    private boolean isPrepared = false;

    public TensorflowLiteModel(String name, File modelFile, boolean quantizeInput) {
        super(name);

        Interpreter.Options tfliteOptions = new Interpreter.Options();
        configureInterpreter(tfliteOptions);
        this.tfLite = new Interpreter(modelFile, tfliteOptions);

        this.quantizeInput = quantizeInput;
    }

    protected abstract void configureInterpreter(Interpreter.Options options);

    @Override
    public void prepare(Utils.Resolution resolution) {
        outputByteBuffer = ByteBuffer.allocateDirect(resolution.getHeight() * resolution.getWidth() * 4);
        inputByteBuffer = ByteBuffer.allocateDirect(3 * resolution.getHeight() * resolution.getWidth() * 4);
        byteInputPixels = new byte[4 * resolution.getWidth() * resolution.getHeight()];
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
        bitmap.get(0, 0, byteInputPixels);
        // Convert the image to floating point.
        if(quantizeInput) {
            for (int y = 0; y < bitmap.height(); ++y) {
                for (int x = 0; x < bitmap.width(); ++x) {
                    final int pixelIdx = 4 * (x + y * bitmap.width());
                    inputByteBuffer.putFloat(byteInputPixels[pixelIdx] / (float) 255.);
                    inputByteBuffer.putFloat(byteInputPixels[pixelIdx + 1] / (float) 255.);
                    inputByteBuffer.putFloat(byteInputPixels[pixelIdx + 2] / (float) 255.);
                }
            }
        } else {
            for (int y = 0; y < bitmap.height(); ++y) {
                for (int x = 0; x < bitmap.width(); ++x) {
                    final int pixelIdx = 4 * (x + y * bitmap.width());
                    inputByteBuffer.putFloat(byteInputPixels[pixelIdx]);
                    inputByteBuffer.putFloat(byteInputPixels[pixelIdx + 1]);
                    inputByteBuffer.putFloat(byteInputPixels[pixelIdx + 2]);
                }
            }
        }
    }
}
