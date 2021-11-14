package org.hermitsocialclub.opmodes.pandemicpanic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import org.hermitsocialclub.util.MoveUtils;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Mecanum Base Op Basic", group = "Hermit")

public class MecanumBaseOpBasic extends LinearOpMode {

    private final PersistantTelemetry pt = new PersistantTelemetry(telemetry);
    public DcMotor left_drive = null;
    public DcMotor left_drive_2 = null;
    public DcMotor right_drive = null;
    public DcMotor right_drive_2 = null;
    public DcMotor[] drive_Motors;
    ElapsedTime runtime = new ElapsedTime();
    private boolean lastAMash = false;
    private boolean lastBMash = false;
    public boolean precisionMode = false;
    public double precisionModifier = 1.25;
    public double invertedControls = 1;
    double clamperPosition = 0;
    double topClawPosition = 0;
    private final boolean foundationPull = false;
    private double initialLeftTicks, initialRightTicks, initialTopTicks;


    @Override
    public void runOpMode() {
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        left_drive_2 = hardwareMap.get(DcMotor.class, "left_drive_2");
        right_drive_2 = hardwareMap.get(DcMotor.class, "right_drive_2");

        left_drive.setDirection(DcMotor.Direction.FORWARD);    // Set to REVERSE if using AndyMark motors
        right_drive.setDirection(DcMotor.Direction.REVERSE);   // Set to FORWARD if using AndyMark motors
        left_drive_2.setDirection(DcMotor.Direction.FORWARD);  // Set to REVERSE if using AndyMark motors
        right_drive_2.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors

        left_drive.setPower(0);
        right_drive.setPower(0);
        left_drive_2.setPower(0);
        right_drive_2.setPower(0);

        drive_Motors = new DcMotor[]{left_drive, right_drive, left_drive_2, right_drive_2};

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_drive_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_drive_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


        telemetry.speak("Hola. Cómo estás?", "spa", "mx");

        while (opModeIsActive()) {

            if (!lastAMash && gamepad1.a) {

                if (precisionMode) {
                    precisionMode = false;
                    precisionModifier = 1.2;
                    pt.setData("Precision Mode", "DEACTIVATED!");

                } else {
                    precisionMode = true;
                    precisionModifier = 0.5;
                    pt.setData("Precision Mode", "ACTIVATED!");

                }
                runtime.reset();

            }
            lastAMash = gamepad1.a;

            if (!lastBMash && gamepad1.b) {
                if (invertedControls == 1) {
                    invertedControls = -1;
                    pt.setData("Inverse Controls", "ACTIVATED!");
                } else if (invertedControls == -1) {
                    invertedControls = 1;
                    pt.setData("Inverse Controls", "DEACTIVATED");
                }
            }
            lastBMash = gamepad1.b;

            double r = MoveUtils.joystickXYToRadius(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = MoveUtils.joystickXYToAngle(gamepad1.left_stick_x, gamepad1.left_stick_y);

            double[] powers = MoveUtils.theAlgorithm(r, robotAngle, -gamepad1.right_stick_x, precisionModifier * invertedControls);
            MoveUtils.setEachMotor(new DcMotor[]{left_drive, right_drive, left_drive_2, right_drive_2}, powers);


        }

    }


}
