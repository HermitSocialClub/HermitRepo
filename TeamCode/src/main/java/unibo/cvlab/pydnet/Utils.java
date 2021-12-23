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

public class Utils {

    public enum Scale {
        FULL(1.f), HALF(0.5f), QUARTER(0.25f), HEIGHT(0.125f);

        private final float value;

        Scale(float value) {
            this.value = value;
        }

        public float getValue() {
            return this.value;
        }

        public String toString() {
            switch (this) {
                case FULL:
                    return "Full";
                case HALF:
                    return "Half";
                case QUARTER:
                    return "Quarter";
                case HEIGHT:
                    return "Height";
            }
            return "Not valid resolution";
        }
    }

    public enum Resolution {
        RES5(384, 640);

        private final int width;
        private final int height;

        Resolution(int width, int height) {
            this.width = width;
            this.height = height;
        }

        public String toString() {
            return "" + this.width + "x" + this.height;
        }

        public int getWidth() {
            return this.width;
        }

        public int getHeight() {
            return this.height;
        }
    }
}
