package unibo.cvlab.pydnet;

import java.io.File;

public class PydnetPP extends TensorflowLiteModel {
    public PydnetPP(File modelFile) {
        super("Pydnet++", modelFile);
        addInputNode("image", "im0");
        addOutputNodes(Utils.Scale.FULL, "truediv");
        addValidResolution(Utils.Resolution.RES5);
    }
}
