package org.hermitsocialclub.legacy;

import android.util.Pair;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import com.vuforia.Vec3F;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.hermitsocialclub.opmodes.pandemicpanic.MecanumConfiguration;
import org.hermitsocialclub.telecat.PersistantTelemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;

import static java.lang.Math.cos;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.hermitsocialclub.legacy.VuforiaConfig.*;

@SuppressWarnings(value = "defaultLocale")
public class SkystoneVuforiaEngine implements IVuforiaEngine {

    public volatile boolean hasAlreadyBeenInit = false;

    //Constants
    public static final double countsPerCentimetre = (countsPerRevolution / driveGearReduction) / (wheelDiameter * Math.PI);
    public static final float mmTargetHeight = (6);
    public static final float stoneZ = 2.00f;
    public static final float bridgeZ = 6.42f;
    public static final float bridgeY = 23;
    public static final float bridgeX = 5.18f;
    public static final float bridgeRotY = 59;
    public static final float bridgeRotZ = 180;
    public static final float halfField = 72;
    public static final float quadField = 36;

    //Instance Variables
    VuforiaTrackables targetsSkyStone;
    private final PersistantTelemetry telemetry;
    public final MecanumConfiguration robot = new MecanumConfiguration();
    private HardwareMap hwMap;
    private VuforiaLocalizer vuforia = null;
    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private final float phoneZRotate = -90;
    protected Orientation orientation;
    protected double xangle;
    protected BNO055IMU imu;
    double[] pos = {23, -halfField};
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    public double PositionPrecision = 0.5;
    private double[] angles;
    private final ElapsedTime runTime = new ElapsedTime();

    public enum setPositionConstancy {X, Y, XY, NONE}


    //Userful Variables
    private final ConcurrentHashMap<String, VuforiaTrackableDefaultListener> iSeeYou = new ConcurrentHashMap<String, VuforiaTrackableDefaultListener>(15);


    /*
     *
     *
     *
     *
     *
     *
     *
     */
    private SkystoneVuforiaEngine(PersistantTelemetry persistantTelemetry) {
        this.telemetry = persistantTelemetry;
    }

    public static SkystoneVuforiaEngine get(PersistantTelemetry pt) {
        return new SkystoneVuforiaEngine(pt);
    }

    @Override
    public void init(final HardwareMap ahwMap) {
        if (!hasAlreadyBeenInit) {
            new Thread(() -> {
                try {
                    long startTime = System.currentTimeMillis();
                    hwMap = ahwMap;
                    robot.init(hwMap);
                    int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
                    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                    parameters.vuforiaLicenseKey = VUFORIA_KEY;
                    parameters.cameraDirection = CAMERA_CHOICE;
                    vuforia = ClassFactory.getInstance().createVuforia(parameters);

                    //Vuforia Focusing
                    CameraDevice.getInstance().setFlashTorchMode(true);
                    CameraDevice.getInstance().setFocusMode(CameraDevice.FOCUS_MODE.FOCUS_MODE_CONTINUOUSAUTO);

                    //Targets
                    targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");
                    VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
                    stoneTarget.setName("Stone Target");
                    VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
                    blueRearBridge.setName("Blue Rear Bridge");
                    VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
                    redRearBridge.setName("Red Rear Bridge");
                    VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
                    redFrontBridge.setName("Red Front Bridge");
                    VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
                    blueFrontBridge.setName("Blue Front Bridge");
                    VuforiaTrackable red1 = targetsSkyStone.get(5);
                    red1.setName("Red Perimeter 1"); //Image 3
                    VuforiaTrackable red2 = targetsSkyStone.get(6);
                    red2.setName("Red Perimeter 2"); //Image 5
                    VuforiaTrackable front1 = targetsSkyStone.get(7);
                    front1.setName("Front Perimeter 1"); //Image 1
                    VuforiaTrackable front2 = targetsSkyStone.get(8);
                    front2.setName("Front Perimeter 2"); //Image 2
                    VuforiaTrackable blue1 = targetsSkyStone.get(9);
                    blue1.setName("Blue Perimeter 1"); //
                    VuforiaTrackable blue2 = targetsSkyStone.get(10);
                    blue2.setName("Blue Perimeter 2");
                    VuforiaTrackable rear1 = targetsSkyStone.get(11);
                    rear1.setName("Rear Perimeter 1");
                    VuforiaTrackable rear2 = targetsSkyStone.get(12);
                    rear2.setName("Rear Perimeter 2");

                    /*List<VuforiaTrackable>*/
                    allTrackables = new ArrayList<VuforiaTrackable>();
                    allTrackables.addAll(targetsSkyStone);

                    stoneTarget.setLocation(OpenGLMatrix
                            .translation(0, 0, stoneZ)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
                    blueFrontBridge.setLocation(OpenGLMatrix
                            .translation(-bridgeX, bridgeY, bridgeZ)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));
                    blueRearBridge.setLocation(OpenGLMatrix
                            .translation(-bridgeX, bridgeY, bridgeZ)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));
                    redFrontBridge.setLocation(OpenGLMatrix
                            .translation(-bridgeX, -bridgeY, bridgeZ)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));
                    redRearBridge.setLocation(OpenGLMatrix
                            .translation(bridgeX, -bridgeY, bridgeZ)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

                    //Set the position of the perimeter targets with relation to origin (center of field)
                    red1.setLocation(OpenGLMatrix
                            .translation(quadField, -halfField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
                    red2.setLocation(OpenGLMatrix
                            .translation(-quadField, -halfField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
                    front1.setLocation(OpenGLMatrix
                            .translation(-halfField, -quadField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
                    front2.setLocation(OpenGLMatrix
                            .translation(-halfField, quadField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
                    blue1.setLocation(OpenGLMatrix
                            .translation(-quadField, halfField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
                    blue2.setLocation(OpenGLMatrix
                            .translation(quadField, halfField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
                    rear1.setLocation(OpenGLMatrix
                            .translation(halfField, quadField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
                    rear2.setLocation(OpenGLMatrix
                            .translation(halfField, -quadField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

                    if (CAMERA_CHOICE == BACK) {
                        phoneYRotate = -90;
                    } else {
                        phoneYRotate = 90;
                    }
                    if (PHONE_IS_PORTRAIT) {
                        phoneXRotate = 90;
                    }

                    // Next, translate the camera lens to where it is on the robot.
                    // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
                    float CAMERA_FORWARD_DISPLACEMENT = 0f;   // eg: Camera is 4 Inches in front of robot center
                    float CAMERA_VERTICAL_DISPLACEMENT = 6.125f;   // eg: Camera is 8 Inches above ground
                    float CAMERA_LEFT_DISPLACEMENT = -6f;     // eg: Camera is ON the robot's center line

                    OpenGLMatrix robotFromCamera = OpenGLMatrix
                            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));


                    for (VuforiaTrackable trackable : allTrackables) {
                        ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
                    }

                    //Store the listeners for later use.
                    for (int i = 0; i < targetsSkyStone.size(); i++) {
                        iSeeYou.put(targetsSkyStone.get(i).getName(), (VuforiaTrackableDefaultListener) targetsSkyStone.get(i).getListener());
                    }

                    targetsSkyStone.activate();
                    telemetry.setData("hmm", "YES! YES! YES! YES!");
                    telemetry.setData("SVE_INIT", "Done in %.2f seconds!", ((double) (System.currentTimeMillis() - startTime)) / 1000D);
                    hasAlreadyBeenInit = true;
                } catch (Exception e) {
                    telemetry.setData("SVE_INIT", "Could not start Vuforia: " + DataDump.dump(e));
                }
            }).start();

        } else {
            telemetry.setData("hmm", "YES! YES! YES! YES!");
        }
    }

    public void debug() {

    }

    @Override
    @SuppressWarnings(value = "unused")
    public OpenGLMatrix getPose(String id) {
        if (!iSeeYou.containsKey(id)) throw new NullPointerException("Target doesn't exist");
        OpenGLMatrix pose = iSeeYou.get(id).getVuforiaCameraFromTarget();
        return pose;
    }

    @Override
    public Vec3F getPosition(String id) {
        if (!iSeeYou.containsKey(id)) throw new NullPointerException("Target doesn't exist");
        OpenGLMatrix pose = getPose(id);
        if (pose == null) return new Vec3F(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
        else return new Vec3F(pose.get(0, 3), pose.get(1, 3), pose.get(2, 3));
    }

    @Override
    public Double getDistanceToObject(String id) {
        Double distance;
        Vec3F position = getPosition(id);
        telemetry.setDebug("VUF_DEBUG0", String.format(
                "position Vec3f: %s",
                Arrays.toString(position.getData())
        ));
        distance = Math.sqrt(
                position.getData()[0] * position.getData()[0] +
                        position.getData()[1] * position.getData()[1] +
                        position.getData()[2] * position.getData()[2]);
        telemetry.setDebug("VUF_DEBUG1", String.format(
                "Distance to %s %.3f",
                id, distance
        ));
        return distance;
    }

    public double getDistanceY(String id) {
        float objectX = Float.POSITIVE_INFINITY;
        Vec3F position = getPosition(id);
        telemetry.setDebug("VUF_DEBUG0", String.format(
                "position Vec3f: %s",
                Arrays.toString(position.getData())
        ));
        objectX = position.getData()[0];
        telemetry.setDebug("VUF_DEBUG1", String.format(
                "X coordinate of %s %.3f",
                id, objectX
        ));
        return objectX;
    }

    public double getDistanceX(String id) {
        float objectY = Float.POSITIVE_INFINITY;
        Vec3F position = getPosition(id);
        telemetry.setDebug("VUF_DEBUG0", String.format(
                "position Vec3f: %s",
                Arrays.toString(position.getData())
        ));
        objectY = position.getData()[2];
        telemetry.setDebug("VUF_DEBUG1", String.format(
                "X coordinate of %s %.3f",
                id, objectY
        ));
        return objectY;
    }

    public Pair<Pose2d, Boolean> scanPose() {
        Pose2d pose = new Pose2d();
        VuforiaTrackable t = allTrackables.get(0);
        double heading = 0;
        float objectX = Float.POSITIVE_INFINITY;
        Vec3F position = new Vec3F();
        String id = "";
        boolean foundTarget = false;
        for (VuforiaTrackable trackable : allTrackables) {
            position = getPosition(trackable.getName());
            heading = get_Orientation()[0];
            if (position.getData()[2] != Double.POSITIVE_INFINITY) {
                foundTarget = true;
                t = trackable;
                break;
            }
        }
        if (foundTarget) {
            position.setData(
                    new float[]{position.getData()[2] + t.getLocation().get(2, 3),
                            position.getData()[0] + t.getLocation().get(0, 3)});
            pose = new Pose2d(position.getData()[0], position.getData()[1], heading);
        }
        telemetry.setDebug("VUF_DEBUG0", String.format(
                "position Vec3f: %s",
                Arrays.toString(position.getData())
        ));
        objectX = position.getData()[0];
        telemetry.setDebug("VUF_DEBUG1", String.format(
                "X coordinate of %s %.3f",
                id, objectX
        ));
        return new Pair<>(pose, foundTarget);
    }

    public void setPosition(double x2, double y2, double speed, List<VuforiaTrackable> allTrackables, setPositionConstancy c) {
        {
            robot.left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.left_drive_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right_drive_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double v1 = 0;
        double v2 = 0;
        double v3 = 0;
        double v4 = 0;
        double theta;
        debugFindLocation(allTrackables);
        switch (c) {
            case X: {
                debugFindLocation(allTrackables);
                x2 = getPos()[0];
                break;
            }
            case Y: {
                debugFindLocation(allTrackables);
                y2 = getPos()[1];
                break;
            }
            case XY: {
                debugFindLocation(allTrackables);
                x2 = getPos()[0];
                y2 = getPos()[1];
                break;
            }
            case NONE: {
                break;
            }

        }
        while (Math.sqrt(Math.pow((y2 - getPos()[1]), 2) + Math.pow((x2 - getPos()[0]), 2)) >= PositionPrecision) {
            debugFindLocation(allTrackables);
            switch (c) {
                case X: {
                    debugFindLocation(allTrackables);
                    x2 = getPos()[0];
                    break;
                }
                case Y: {
                    debugFindLocation(allTrackables);
                    y2 = getPos()[1];
                    break;
                }
                case XY: {
                    debugFindLocation(allTrackables);
                    x2 = getPos()[0];
                    y2 = getPos()[1];
                    break;
                }
                case NONE: {
                    break;
                }

            }
            if (!isTargetFound(allTrackables)) {
                robot.left_drive.setPower(0);
                robot.right_drive.setPower(0);
                robot.left_drive_2.setPower(0);
                robot.right_drive_2.setPower(0);
                v1 = 0;
                v2 = 0;
                v3 = 0;
                v4 = 0;
            } else {
                theta = Math.atan2((y2 - getPos()[1]), (x2 - getPos()[0]));
                v1 = cos(theta) * speed;
                v2 = Math.sin(theta) * speed;
                v3 = Math.sin(theta) * speed;
                v4 = cos(theta) * speed;
                robot.left_drive.setPower(0);
                robot.right_drive.setPower(0);
                robot.left_drive_2.setPower(0);
                robot.right_drive_2.setPower(0);
            }
            telemetry.setDebug("MOTOR_POWERS", "V1: %f, V2: %f, V3: %f, V4: %f", v1, v2, v3, v4);
            telemetry.setDebug("CURRENT_POSITION", "X: %f, Y: % f", getPos()[0], getPos()[1]);
            telemetry.setDebug("TARGET_POSITION", "X: %f, Y: %f", x2, y2);
            telemetry.setDebug("DISTANCE_REMAINING", Math.sqrt(Math.pow((y2 - getPos()[1]), 2) + Math.pow((x2 - getPos()[0]), 2)));
            telemetry.setDebug("TARGET_VISIBLE", isTargetFound(allTrackables));

        }
    }

    public void setPositionColor(double x2, double y2, double speed, List<VuforiaTrackable> allTrackables, setPositionConstancy c) {
        {
            robot.left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.left_drive_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right_drive_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        double v1;
        double v2;
        double v3;
        double v4;
        double theta;
        switch (c) {
            case X: {
                debugFindLocation(allTrackables);
                x2 = getPos()[0];
                break;
            }
            case Y: {
                debugFindLocation(allTrackables);
                y2 = getPos()[1];
                break;
            }
            case XY: {
                debugFindLocation(allTrackables);
                x2 = getPos()[0];
                y2 = getPos()[1];
                break;
            }
            case NONE: {
                break;
            }

        }
        while (Math.sqrt(Math.pow((y2 - getPos()[1]), 2) + Math.pow((x2 - getPos()[0]), 2)) >= PositionPrecision
                && robot.colorSensor.green() >= 50) {
            debugFindLocation(allTrackables);
            switch (c) {
                case X: {
                    debugFindLocation(allTrackables);
                    x2 = getPos()[0];
                    break;
                }
                case Y: {
                    debugFindLocation(allTrackables);
                    y2 = getPos()[1];
                    break;
                }
                case XY: {
                    debugFindLocation(allTrackables);
                    x2 = getPos()[0];
                    y2 = getPos()[1];
                    break;
                }
                case NONE: {
                    break;
                }

            }
            theta = Math.atan2((y2 - getPos()[1]), (x2 - getPos()[0]));
            v1 = cos(theta) * speed;
            v2 = Math.sin(theta) * speed;
            v3 = Math.sin(theta) * speed;
            v4 = cos(theta) * speed;
            robot.left_drive.setPower(v1);
            robot.right_drive.setPower(v2);
            robot.left_drive_2.setPower(v3);
            robot.right_drive_2.setPower(v4);
            telemetry.setDebug("CURRENT_POSITION", "X: %f, Y: % f", getPos()[0], getPos()[1]);
            telemetry.setDebug("TARGET_POSITION", "X: %f, Y: %f", x2, y2);
            telemetry.setDebug("DISTANCE_REMAINING", Math.sqrt(Math.pow((y2 - getPos()[1]), 2) + Math.pow((x2 - getPos()[0]), 2)));
            telemetry.setDebug("RGB_DETECTION", "RED: %f, ");
        }
        telemetry.setData("Movement", "Finished");
    }

    public void debugFindLocation(List<VuforiaTrackable> allTrackables) {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        boolean done = false;
        while (!done) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.setData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.setData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        -translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, -translation.get(2) / mmPerInch);
                setPos(new double[]{-translation.get(0) / mmPerInch, translation.get(1) / mmPerInch});
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.setData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                done = true;
            } else {
                telemetry.setData("Visible Target", "none");
                done = true;
            }
        }
    }

    public boolean isTargetFound(List<VuforiaTrackable> allTrackables) {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        boolean done = false;
        while (!done) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() && trackable.getName() != "Stone Target") {
                    telemetry.setData("Visible Target", trackable.getName());
                    targetVisible = true;
                    done = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            targetVisible = false;
            done = true;
            telemetry.setData("Continuously", "Looping");
            // Provide feedback as to where the robot is located (if we know).
        }
        return targetVisible;
    }

    public double[] positionAverage(long sampleDuration) {
        debugFindLocation(getallTrackables());
        Double[] xPos = {getPos()[0]};
        Double[] yPos = {getPos()[1]};
        double[] positionAverage;
        List<Double> xPosList = new ArrayList<Double>(Arrays.asList(xPos));
        List<Double> yPosList = new ArrayList<Double>(Arrays.asList(yPos));

        runTime.reset();
        while (runTime.milliseconds() <= sampleDuration) {
            debugFindLocation(getallTrackables());
            xPosList.add(getPos()[0]);
            yPosList.add(getPos()[1]);
        }
        xPosList.toArray();
        yPosList.toArray();
        double xPosSum = 0;
        double yPosSum = 0;
        for (int i = 0; i < xPos.length; i++) {
            xPosSum += xPos[i];
            yPosSum += yPos[i];
        }
        positionAverage = new double[]{xPosSum / xPos.length, yPosSum / yPos.length};
        return positionAverage;
    }
    /* public void find_Stone_Target(List<VuforiaTrackable> allTrackables){
        double acquisitionLocationX;
        double acquisitionLocationY;
         while(robot.colorSensor.green() >= 50 || xpos > -halfField){
             robot.right_drive.setPower(.2);
             robot.right_drive_2.setPower(-.2);
             robot.left_drive.setPower(-.2);
             robot.left_drive_2.setPower(.2);}
         stopMoving();
         debugFindLocation(allTrackables);
         VectorF stone_Target = lastLocation.getTranslation();
         acquisitionLocationX = stone_Target.get(0)-(stone_Target_Middle)+(colorSensorDistanceFromIntake);
         acquisitionLocationY = stone_Target.get(1)-(2);
         setPosition(acquisitionLocationX,acquisitionLocationY,.4,allTrackables);
     }*/

    public void getLocation(List<VuforiaTrackable> allTrackables) {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.setData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.setData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0), translation.get(1), translation.get(2));
            setPos(new double[]{translation.get(0), translation.get(1)});

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.setData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.setData("Visible Target", "none");
        }

    }

    public double[] get_Orientation() {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        boolean done = false;
        while (!done) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.setData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.setData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        -translation.get(0) / mmPerInch, -translation.get(1) / mmPerInch, -translation.get(2) / mmPerInch);
                setPos(new double[]{-translation.get(0) / mmPerInch, translation.get(1) / mmPerInch});
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.setData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                angles = new double[]{rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle};
                done = true;
            } else {
                telemetry.setData("Visible Target", "none");
            }
        }
        return angles;
    }

    public double[] getPos() {
        return this.pos;
    }

    public void setPos(double[] newPos) {
        this.pos = newPos;
    }

    public List<VuforiaTrackable> getallTrackables() {
        return this.allTrackables;
    }

    public AutoUtils.SkystoneNumber findBlock() {
        Size size = new Size(3266, 2450);
        CameraCaptureRequest c;
        AutoUtils.SkystoneNumber block = null;
        VuforiaTrackableDefaultListener v = (VuforiaTrackableDefaultListener) allTrackables.get(0).getListener();
        OpenGLMatrix locationOnScreen = v.getPose();
        try {
            c = vuforia.getCamera().createCaptureRequest(1, size, 24);
        } catch (CameraException e) {
            telemetry.setDebug("No", "You");
        }
        locationOnScreen.getData();
        return block;
    }

    public boolean hasInit() {
        return hasAlreadyBeenInit;
    }

}
