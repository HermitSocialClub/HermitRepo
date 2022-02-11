package org.hermitsocialclub.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

public class LinearHelpers {
    private BaselineMecanumDrive drive;
    private PersistantTelemetry telemetry;
    private MotorConfigurationType liftType;
    private ElapsedTime runtime = new ElapsedTime();
    public int startingPosition;
    public int currentPosition;
    public final static int TICKS_PER_REV = 652;
    private int targetLevel = 0;
    private int currentLevel = 0;

    public final static int INCREMENT = 5;

    public enum LEVEL{
        ZERO(0),ONE(800),TWO(1600),
        THREE(2200),FOUR(2800);

        LEVEL(int targetPosition){
            this.targetPosition = targetPosition;
        }
        public int targetPosition;
    }

    public enum STATE{

        UP, DOWN, SAME, SET

    }

    public enum MODE{

        TELEOP, AUTON

    }

    private LEVEL targetLevelNew = LEVEL.ZERO;
    private LEVEL currentLevelNew = LEVEL.ZERO;

    private STATE state = STATE.SAME;

    private MODE mode = MODE.AUTON;

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
//    public void LiftLinears () {
//
//        if (targetLevel.ordinal() < 4) this.targetLevel.ordinal();
//
//    }

    /**
     * Sets the Linears to a Specified Level
     */
    public void setLinears (int level) {
        this.targetLevel = level;
    }

    public void setLevel(LEVEL level){
        this.targetLevelNew = level;
    }

    public void AutomaticLinears ()  {

    }

    public void LinearUpdateNew () {
        switch (mode){
            case AUTON: {
                state = (targetLevelNew.ordinal() > currentLevelNew.ordinal()) ? STATE.UP :
                        (targetLevelNew.ordinal() < currentLevelNew.ordinal()) ? STATE.DOWN : STATE.SAME;
                break;
            }
            case TELEOP: {

                switch (state){

                    case UP: {
                        if (drive.lift.getCurrentPosition() + 5 >
                                LEVEL.FOUR.targetPosition + startingPosition){
                            drive.lift.setTargetPosition(LEVEL.FOUR.targetPosition);
                            drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            drive.lift.setPower(.95);
                            break;
                        }
                        drive.lift.setTargetPosition(drive.lift.getCurrentPosition()
                        + INCREMENT);
                        drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.lift.setPower(.95);

                        break;
                    }
                    case DOWN: {
                        if (drive.linearSwitch.isPressed()){
                            drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            drive.lift.setPower(0);
                            startingPosition = drive.lift.getCurrentPosition();
                            break;
                        }
                        drive.lift.setTargetPosition(drive.lift.getCurrentPosition()
                                - INCREMENT);
                    }

                }

                break;
            }
        }


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
        if (targetLevel != currentLevel) {
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
               drive.lift.setTargetPosition(startingPosition + 2200);
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
           else if (targetLevel == 1) {
                runtime.reset();
                telemetry.setData("Going to Level: ", targetLevel);
                drive.lift.setTargetPosition(startingPosition + 800);
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

    public void setMode(MODE mode){
        this.mode = mode;
    }

    public void setState(STATE state){
        this.state = state;
    }

}
