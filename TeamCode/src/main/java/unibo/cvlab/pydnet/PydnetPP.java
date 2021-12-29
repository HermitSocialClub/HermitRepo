package unibo.cvlab.pydnet;

import org.tensorflow.lite.Interpreter;

import java.io.File;

public class PydnetPP extends TensorflowLiteModel {
    public PydnetPP(File modelFile) {
        super("Pydnet++", modelFile, true);
        addInputNode("image", "im0");
        addOutputNodes(Utils.Scale.FULL, "truediv");
        addValidResolution(Utils.Resolution.RES5);
    }

    @Override
    protected void configureInterpreter(Interpreter.Options options) {
        // <Aditya> also make sure when you init pydnet with tflite to use xnnpack
        options.setUseXNNPACK(true);
    }
}
