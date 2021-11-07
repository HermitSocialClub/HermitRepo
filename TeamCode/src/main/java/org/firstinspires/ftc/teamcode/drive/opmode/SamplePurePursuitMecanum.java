package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Iterative;

/*
 * Sample Pure Pursuit Drive to a Point based on Gluten Free tutorial.
 */
@Config
public class SamplePurePursuitMecanum extends BasicOpMode_Iterative {


   public void init() {

   }







    public void loop() {
        RobotMovement.goToPosition(358/2, 358/2, 0.3);
    }
}
