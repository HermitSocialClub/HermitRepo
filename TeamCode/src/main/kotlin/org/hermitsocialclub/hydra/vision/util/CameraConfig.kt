package org.hermitsocialclub.hydra.vision.util

import org.hermitsocialclub.hydra.vision.util.VisionUtils.XMLTagDSL.Companion.parseXML
import org.opencv.core.CvType.CV_64F
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import java.io.File
import java.lang.IllegalArgumentException

/**
 * Camera calibration constants. For how to get these, see
 * [this tutorial](https://docs.opencv.org/master/d4/d94/tutorial_camera_calibration.html)
 * and use
 * [this program](https://github.com/opencv/opencv/tree/master/samples/cpp/tutorial_code/calib3d/camera_calibration).
 *
 * You'll need to compile OpenCV in order to run the example calibrator. Clone the repo
 * and install CMake and Visual Studio (only the dev tools are required, not the full IDE).
 * Then run
 * ```
 * mkdir build
 * cd build
 * cmake -D CMAKE_BUILD_TYPE=RELEASE -D BUILD_PYTHON_SUPPORT=OFF -D BUILD_EXAMPLES=ON ..
 * MSBuild /p:Configuration=Release samples/cpp/example_tutorial_camera_calibration.vcxproj
 * ```
 * to build `example_tutorial_camera_calibration.exe`.
 *
 * Using MinGW will not work since some of OpenCV's dependencies use C++ 14 features that
 * are not implemented in MinGW.
 * Clang/GCC probably works on Linux and MacOS but I haven't tested
 *
 * &#8211; Arc
 */
data class CameraConfig(var cameraMatrix: Mat, var distortionCoefficients: MatOfDouble) {
    companion object {
        @Deprecated("does not work right now")
        @JvmStatic
        fun loadFromFile(file: File): CameraConfig {
            if(!file.exists()) {
                throw RuntimeException("Camera config file does not exist!")
            }
            var cameraMatrix: Mat? = null
            var distortionCoefficients: MatOfDouble? = null
            parseXML(file) {
                tag("opencv_storage") {
                    tag("camera_matrix") {
                        tag("data") {
                            whenParsed {
                                cameraMatrix = Mat.zeros(3, 3, CV_64F)
                                parseMat(cameraMatrix!!, parser.text)
                            }
                        }
                    }
                    tag("distortion_coefficients") {
                        tag("rows") {
                            whenParsed {
                                distortionCoefficients = MatOfDouble(Mat.zeros(parser.text.toInt(), 1, CV_64F))
                            }
                        }
                        tag("data") {
                            whenParsed {
                                parseMat(distortionCoefficients!!, parser.text)
                            }
                        }
                    }
                }
            }
            return CameraConfig(cameraMatrix!!, distortionCoefficients!!)
        }

        private fun parseMat(mat: Mat, string: String) {
            var row = 0
            var col = 0
            for (value in string.split(" ")) {
                mat.put(row, col, value.toDouble())
                col++
                if (col >= mat.width()) {
                    col = 0
                    row++
                    if (row >= mat.height()) {
                        throw IllegalArgumentException("too many values")
                    }
                }
            }
        }
    }
}