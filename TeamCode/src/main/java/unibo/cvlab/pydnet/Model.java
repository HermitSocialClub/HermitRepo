/*
Copyright 2019 Filippo Aleotti
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
* Author: Filippo Aleotti
* Mail: filippo.aleotti2@unibo.it
*/

package unibo.cvlab.pydnet;

import android.graphics.Bitmap;
import org.opencv.core.Mat;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class Model {

    protected Map<Utils.Scale, String> outputNodes;
    protected List<Utils.Resolution> validResolutions;
    protected String name;
    protected HashMap<String, float[]> results;
    protected HashMap<String, String> inputNodes;

    public Model(String name) {
        this.name = name;
        this.outputNodes = new HashMap<>();
        this.inputNodes = new HashMap<>();
        this.validResolutions = new ArrayList<>();
        this.results = new HashMap<>();
    }

    public void addOutputNodes(Utils.Scale scale, String node) {
        this.outputNodes.put(scale, node);
    }

    public void addInputNode(String name, String node) {
        if (!this.inputNodes.containsKey(name))
            this.inputNodes.put(name, node);
    }

    public void addValidResolution(Utils.Resolution resolution) {
        if (!this.validResolutions.contains(resolution))
            this.validResolutions.add(resolution);
    }

    public String getInputNode(String name) {
        return this.inputNodes.get(name);
    }

    public abstract void prepare(Utils.Resolution resolution);

    public abstract FloatBuffer doInference(Mat input, Utils.Scale scale);
}
