package unibo.cvlab.pydnet;

import java.io.File;

public class PydnetPP extends TensorflowLiteModel {
    public PydnetPP(File modelFile) {
        super("Pydnet++", modelFile);
        addInputNode("image", "im0");
        addOutputNodes(Utils.Scale.HALF, "PSD/resize_images/ResizeBilinear");
        addOutputNodes(Utils.Scale.QUARTER, "PSD/resize_images_1/ResizeBilinear");
        addOutputNodes(Utils.Scale.HEIGHT, "PSD/resize_images_2/ResizeBilinear");
        addValidResolution(Utils.Resolution.RES4);
    }
}
