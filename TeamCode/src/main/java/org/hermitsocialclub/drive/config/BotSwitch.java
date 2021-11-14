package org.hermitsocialclub.drive.config;

public class BotSwitch {

    static Meet3BotConstants meet3Bot;
    static Meet0BotConstants meet0Bot;

    //TODO: If you add another bot remember to add it to @BaselineMecanumDrive.java too
    public enum BOT {
        MEET3BOT(meet3Bot), MEET0BOT(meet0Bot);
        public DriveConstants constants;

        BOT(DriveConstants constants) {
            this.constants = constants;
        }
    }

    public static BOT bot;

    public BotSwitch() {
        bot = BOT.MEET0BOT;
    }
}
