package org.hermitsocialclub.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.hermitsocialclub.drive.BaselineMecanumDrive;
import org.hermitsocialclub.telecat.PersistantTelemetry;

public class LinearHelpers {
    private BaselineMecanumDrive drive;
    private PersistantTelemetry telemetry;
    private MotorConfigurationType liftType;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime gameTime;
    public int startingPosition;
    public int currentPosition;
    public final static int TICKS_PER_REV = 652;
    private int targetLevel = 0;
    private int currentLevel = 0;
    public int levelOverride = 0;

    public final static int INCREMENT = 400;

    public enum LEVEL{
        ZERO(0),ONE(950),TWO(1500),
        THREE(2550),FOUR(3200);

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
    @Deprecated
    public LinearHelpers(BaselineMecanumDrive drive, PersistantTelemetry telemetry){
        this.telemetry = telemetry;
        this.drive = drive;
        this.liftType = drive.lift.getMotorType();
        startingPosition = drive.lift.getCurrentPosition();
        currentPosition = drive.lift.getCurrentPosition();
        drive.lift.setTargetPositionTolerance(20);
    }
    public LinearHelpers(BaselineMecanumDrive drive, PersistantTelemetry telemetry, ElapsedTime gameTime){
        this.telemetry = telemetry;
        this.drive = drive;
        this.gameTime = gameTime;
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
        telemetry.setData("Linear State: ", state.toString());
        switch (mode){
            case AUTON: {
                RobotLog.d("Linear Case: AUTON");
                state = (targetLevelNew.ordinal() > currentLevelNew.ordinal()) ? STATE.UP :
                        (targetLevelNew.ordinal() < currentLevelNew.ordinal()) ? STATE.DOWN : STATE.SAME;
                switch (state){
                    case UP: {
                        drive.lift.setTargetPosition(startingPosition
                                + targetLevelNew.targetPosition);
                        drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.lift.setPower(.95);
                        if (Math.abs(drive.lift.getCurrentPosition()
                                - (targetLevelNew.targetPosition + startingPosition))
                                < 5){
                            currentLevelNew = targetLevelNew;
                        }
                        break;
                    }
                    case DOWN: {
                        if (drive.linearSwitch.isPressed()){
                            drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            drive.lift.setPower(0);
                            startingPosition = drive.lift.getCurrentPosition();
                            currentLevelNew = LEVEL.ZERO;
                            break;
                        }
                        drive.lift.setTargetPosition(startingPosition
                                + targetLevelNew.targetPosition);
                        drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.lift.setPower(.95);
                        if (Math.abs(drive.lift.getCurrentPosition()
                                - (targetLevelNew.targetPosition + startingPosition))
                        < 5){
                            currentLevelNew = targetLevelNew;
                        }
                        break;

                    }
                    case SAME: {
                        drive.lift.setTargetPosition(currentLevelNew.targetPosition);
                        drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.lift.setPower(.95);
                        break;
                    }
                }
                break;
            }
            case TELEOP: {
                RobotLog.d("Linear Case: TELEOP");
                switch (state){

                    case UP: {
                        RobotLog.d("Linear Case: UP TELEOP");

                        if (drive.lift.getCurrentPosition() + INCREMENT >
                                LEVEL.values()[((gameTime.seconds() < 90) ? 3 : 4)].targetPosition
                                        + startingPosition){
                            drive.lift.setTargetPosition(LEVEL.values()
                                    [((gameTime.seconds() + levelOverride < 90) ? 3 : 4)].targetPosition
                                    + startingPosition);
                            drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            drive.lift.setPower(.95);
                            state = STATE.SET;
                            break;
                        }
                        drive.lift.setTargetPosition(drive.lift.getCurrentPosition()
                        + INCREMENT);
                        drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.lift.setPower(.95);

                        break;
                    }
                    case DOWN: {
                        RobotLog.d("Linear Case: DOWN TELEOP");
                        if (drive.linearSwitch.isPressed()){
                            drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            drive.lift.setPower(0);
                            startingPosition = drive.lift.getCurrentPosition();
                            setState(STATE.SAME);
                            break;
                        }
                        drive.lift.setTargetPosition(drive.lift.getCurrentPosition()
                                - INCREMENT);
                        drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.lift.setPower(.95);
                        break;
                    }
                    case SET: {
                        RobotLog.d("Linear Case: SET TELEOP");
                        LEVEL closestLevel = LEVEL.ZERO;
                        int closeness = Math.abs(drive.lift.getCurrentPosition()
                                - (closestLevel.targetPosition + startingPosition));
                        for (int i = 1; i < ((gameTime.seconds() < 90) ? 4 : 5); i++) {
                            int temp = Math.abs(drive.lift.getCurrentPosition()
                                    - (LEVEL.values()[i].targetPosition + startingPosition));
                            if(temp < closeness){
                                closestLevel = LEVEL.values()[i];
                                closeness = temp;
                            }
                        }
                        telemetry.setData("Target Level: ", closestLevel.toString());
                        drive.lift.setTargetPosition(startingPosition + closestLevel.targetPosition);
                        RobotLog.e("Target Position: " + drive.lift.getTargetPosition());
                        drive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.lift.setPower(.95);
                        break;
                    }
                    case SAME: {
                        drive.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        drive.lift.setVelocity(0);
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

    public STATE getState(){
        return state;
    }

}
