package org.hermitsocialclub.opmodes.freightfrenzy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.localizers.T265LocalizerRR;
import org.hermitsocialclub.telecat.PersistantTelemetry;
import static org.hermitsocialclub.drive.config.DriveConstants.*;

@TeleOp(name = "Meet0Tele")
public class Meet0TeleOp extends OpMode {
    Canvas field;
    TelemetryPacket packet;

    private PersistantTelemetry telemetry;

    private BaselineMecanumDrive drive;

    private final FtcDashboard dash = FtcDashboard.getInstance();

    private final int robotRadius = 8;

    private double trigVal = 0;
    //in ticks
    private double liftSpeed = -15;
    private MotorConfigurationType liftType;

    private double intakeSpeed = .85;
    private MotorConfigurationType intakeType;

    public static double liftP = 3.0;
    public static double liftI = 1.0;
    public static double liftD = .1;
    public static double liftF = 22.259453425873854;
    public static PIDFCoefficients liftCoefficients = new PIDFCoefficients(3,1,.1,22.259453425873854);


    @Override

    public void init() {
        telemetry = new PersistantTelemetry(super.telemetry);
        RUN_USING_ENCODER = true;
        drive = new BaselineMecanumDrive(hardwareMap, telemetry);
        drive.setPoseEstimate(new Pose2d(0, 0,0));
        drive.duck_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftType = drive.lift.getMotorType();
        intakeType = drive.intake.getMotorType();
        telemetry.setData("Lift Pos",drive.lift.getCurrentPosition());
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        /*drive.lift.setTargetPosition(drive.lift.getCurrentPosition() +
                (int)(liftSpeed * gamepad1.right_stick_y));
        drive.lift.setPower(.45);*/

        trigVal = -gamepad2.right_stick_y > 0.05 ? -gamepad2.right_stick_y * 1.25 :
                -gamepad2.right_stick_y < -.05 ? -gamepad2.right_stick_y : 0.000;

        drive.lift.setVelocity(liftType
                .getMaxRPM()/60 * liftType.getAchieveableMaxRPMFraction() * .65 *
                trigVal, AngleUnit.RADIANS);


        if (gamepad2.right_trigger > 0.05) {
            drive.intake.setVelocity(intakeSpeed
                    * intakeType.getAchieveableMaxRPMFraction() *
                    intakeType.getMaxRPM()/60 * Math.PI * 2, AngleUnit.RADIANS);
        } else if (gamepad2.left_trigger > .05) {
            drive.intake.setVelocity(-intakeSpeed
                    * intakeType.getAchieveableMaxRPMFraction() *
                    intakeType.getMaxRPM()/60 * Math.PI * 2, AngleUnit.RADIANS);
        } else drive.intake.setPower(0);

        if (Math.abs(gamepad2.left_stick_y) > 0.05) {
            drive.duck_wheel.setPower(0.3 * Math.signum(-gamepad2.left_stick_y));
        } else {
            drive.duck_wheel.setPower(0);
        }

        if(gamepad2.left_bumper){
            telemetry.setData("left_bumper"," pressed");
            drive.outtakeArm.setPosition(.4);
            telemetry.setData("Servo_Pos: ", drive.outtakeArm.getPosition());
        }else {
            telemetry.setData("right_bumper"," pressed");
            drive.outtakeArm.setPosition(1.0);
            telemetry.setData("Servo_Pos: ", drive.outtakeArm.getPosition());
        }
       /* drive.duck_wheel.setVelocity(liftType
                .getAchieveableMaxTicksPerSecond() * .65 *
                trigVal, AngleUnit.RADIANS);
*/
        packet = new TelemetryPacket();

        field = packet.fieldOverlay();
    if(!gamepad1.b) drive.setWeightedDrivePower(
            new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ).times(.55)
    );
        drive.update();

        Pose2d pose = drive.getPoseEstimate();
        double angle = pose.getHeading();
        /*telemetry.setData("x", pose.getX());
        telemetry.setData("y", pose.getY());
        telemetry.setData("heading", pose.getHeading());*/



        field.strokeCircle(pose.getX(), pose.getY(), angle);
        double arrowX = Math.cos(angle) * robotRadius, arrowY = Math.sin(angle) * robotRadius;
        double x1 = pose.getX() + arrowX / 2, y1 = pose.getY() + arrowY / 2;
        double x2 = pose.getX() + arrowX, y2 = pose.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);
        packet.put("Pose", pose.toString());
        packet.put("Pose Confidence", ((T265LocalizerRR) (drive.getLocalizer())).getConfidence());
        telemetry.setData("Pose Estimate",pose);

        dash.sendTelemetryPacket(packet);

    }

    @Override
    public void stop() {
        super.stop();
    }
}
