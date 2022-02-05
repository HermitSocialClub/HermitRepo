package org.hermitsocialclub.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

public class LinearHelpers {
    private BaselineMecanumDrive drive;
    private PersistantTelemetry telemetry;
    private MotorConfigurationType liftType;
    private ElapsedTime runtime = new ElapsedTime();
    private int startingPosition;
    public int currentPosition;
    private int currentLevel = 1;
    private int targetLevel = 1;
//    line

    public LinearHelpers(BaselineMecanumDrive drive, PersistantTelemetry telemetry){
        this.telemetry = telemetry;
        this.drive = drive;
        this.liftType = drive.lift.getMotorType();
        startingPosition = drive.lift.getCurrentPosition();
        currentPosition = drive.lift.getCurrentPosition();
        drive.lift.setTargetPositionTolerance(20);
    }

    /**
     * Lifts the linear to the highest level*/
    public void LiftLinears () {

        if (targetLevel < 4) this.targetLevel++;

    }

    /**
     * Sets the Linears to a Specified Level
     */
    public void setLinears (int level) {
        this.targetLevel = level;
    }

    /**
     * Returns linears to bottom
     */
    public void ReturnLinears () {
        if (targetLevel > 0) this.targetLevel--;
//        Callback()
    }
    public void AutomaticLinears ()  {

    }

    public void LinearUpdate () {
        int i = 0;
       currentPosition = drive.lift.getCurrentPosition();
        telemetry.setData("curPosLift",currentPosition);
        telemetry.setData("targetLevel",targetLevel);
        telemetry.setData("currlevel",currentLevel);
        telemetry.setData("Lift Power", drive.lift.getPower());
        telemetry.setData("a",runtime.seconds());
       if (drive.lift.isBusy()) {
           telemetry.setData("isGoing: ", "Still Going");
           if(targetLevel == currentLevel) return;
       };
       telemetry.setData("isGoing: ","Stopping and Resetting");
//       drive.lift.setPower(0);
//        drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (targetLevel > currentLevel) {
            if (targetLevel == 4) {
                runtime.reset();
                telemetry.setData("Going to Level: ", targetLevel);
                i++;
                telemetry.setData("reset timer: ", i);
                drive.lift.setTargetPosition(startingPosition + 2400);
                telemetry.setData("startingPosition: ", startingPosition);
                telemetry.setData("target: ", drive.lift.getTargetPosition());
                drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.lift.setPower(0.95);
                currentLevel = targetLevel;
            }
            else if (targetLevel == 3) {
               runtime.reset();
               telemetry.setData("Going to Level: ", targetLevel);
               i++;
               telemetry.setData("reset timer: ", i);
               drive.lift.setTargetPosition(startingPosition + 2000);
               telemetry.setData("startingPosition: ", startingPosition);
               telemetry.setData("target: ", drive.lift.getTargetPosition());
               drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               drive.lift.setPower(0.95);
               currentLevel = targetLevel;
           }
           else if (targetLevel == 2) {
               runtime.reset();
               telemetry.setData("Going to Level: ", targetLevel);
               i++;
               telemetry.setData("reset timer: ", i);
               drive.lift.setTargetPosition(startingPosition + 800);
               telemetry.setData("startingPosition: ", startingPosition);
               telemetry.setData("target: ", drive.lift.getTargetPosition());
               drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               drive.lift.setPower(0.95);
               currentLevel = targetLevel;
           }
       }
       else if (targetLevel < currentLevel) {
           if (targetLevel == 2) {
               runtime.reset();
               telemetry.setData("Going to Level: ", targetLevel);
               i++;
               telemetry.setData("reset timer: ", i);
               drive.lift.setTargetPosition(startingPosition + 800);
               telemetry.setData("startingPosition: ", startingPosition);
               telemetry.setData("target: ", drive.lift.getTargetPosition());
               drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               drive.lift.setPower(0.95);
               currentLevel = targetLevel;
           }
           else if (targetLevel == 1) {
               runtime.reset();
               telemetry.setData("Going to Level: ", targetLevel);
               drive.lift.setTargetPosition(startingPosition + 200);
               telemetry.setData("target: ", drive.lift.getTargetPosition());
               telemetry.setData("startingPosition: ", startingPosition);
               drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               drive.lift.setPower(0.95);
               currentLevel = targetLevel;
           }
           else if (targetLevel == 0) {
               runtime.reset();
               telemetry.setData("Going to Level: ", targetLevel);
               drive.lift.setTargetPosition(startingPosition);
               telemetry.setData("target: ", drive.lift.getTargetPosition());
               telemetry.setData("startingPosition: ", startingPosition);
               drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               drive.lift.setPower(0.95);
               currentLevel = targetLevel;
           }
       }
    }
//        boolean firstRun = true;
//        if(drive.linearSwitch.isPressed()) {
//            drive.lift.setPower(0);
//            this.currentlyBottomed = true;
//            firstRun = false;
//        }
//        else if(this.shouldBottom && !this.currentlyBottomed) {
//            drive.lift.setVelocity(liftType
//                    .getMaxRPM() / 60 * liftType.getAchieveableMaxRPMFraction() * -.55 *
//                    1, AngleUnit.RADIANS);
//        }
//        if(!this.shouldBottom && this.currentlyBottomed) {
////            this.currentlyBottomed
//            if (firstRun){
//                time.reset();
//                firstRun = false;
//            }
//            drive.lift.setVelocity(liftType
//                    .getMaxRPM() / 60 * liftType.getAchieveableMaxRPMFraction() * .85 *
//                    1, AngleUnit.RADIANS);
//
//        }
//        if (time.seconds() >= 2.1){
//            drive.lift.setVelocity(liftType
//                    .getMaxRPM() / 60 * liftType.getAchieveableMaxRPMFraction() * .2 *
//                    1);
//            firstRun = true;
//        }
////        else if (this.shouldBottom && this.currentLevvel != 2) {
//
////        }
//    }


}
