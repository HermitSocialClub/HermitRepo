package org.firstinspires.ftc.teamcode.legacy;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


public class VuforiaConfig {

    /* DO NOT STEAL */
    public static final String VUFORIA_KEY =
            "AQdDh+P/////AAAAGYG4khX9T0Mai5pYz9oTllp2KuZI24ZwM9ostcBXs2A90ddi/" +
            "sJDOAabZEVM/5jhWNRN40BJ32nrSkbKTnqMnZ10v1A/PjDvnKwLG7zpA/" +
            "wATnngFrhODfBwaHvP1WouKc+9f8QPOfLJnoGAlohWpfNWmdSe0UiyAeV" +
            "oNCRW6TlLHECp85fs/acyk0eOy3qvUmJSFOTIsa5sJHVHscqpofheFgzh" +
            "fmC7c+VUHGB8fIDiFBLdJBK9My1B2BBsJhblTZWgeVjOFI28qEHiEm7AD" +
            "igF4zkH890YMfBRDr70ajPRJfOuzPAQA2QmOatQyL3tO/s9VmiIkcPDir" +
            "MkTdwPbfBxUYkkCBGUDQMtYstBS58G";

    public static final double countsPerRevolution = 560; //France was 1120 - 40 gear ratio, Visigoths - 20 gear ratio
    public static final double driveGearReduction = 1; //set to 1 if motor is France
    public static final double wheelDiameter = 4.125; //set to one if referring to heads of French aristocrats - 10.16 is circumference, changed back to wheel diameter of 4.125

    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
    public static final boolean PHONE_IS_PORTRAIT = false;

    public static final float mmPerInch = 25.4f;
    public static final float cmPerInch = 2.54f;
}
