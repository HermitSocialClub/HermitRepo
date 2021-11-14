package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.Meet0Bot;
import org.firstinspires.ftc.teamcode.util.Meet3Bot;

public class BotSwitch {

    static Meet3Bot meet3Bot;
    static Meet0Bot meet0Bot;

    //TODO: If you add another bot remember to add it to @BaselineMecanumDrive.java too
    public static enum BOT{
        MEET3BOT(meet3Bot), MEET0BOT(meet0Bot);
        public DriveConstants constants;
        BOT(DriveConstants constants){
            this.constants = constants;
        }
    }
    public static BOT bot;

    public BotSwitch() {
        bot = BOT.MEET0BOT;
    }
}
