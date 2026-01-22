package frc.robot;

/*
USAGE EXAMPLE

ledSubsystem.currentPattern = new double[]{1.0, skyBlue, hotPink, white, hotPink, skyBlue}; //first index is speed in Hz
ledSubsystem.ledPatternState = null; //required: resets anim
ledSubssytem.currentPattern - new double[]{1.0, off}; //turns the LED off
 */



public final class ledConstants {

    public static class Constants {
        public static final int led_pin = 0;
        public static final double hotPink = 0.57; //trans flag colours
        public static final double darkred = 0.59;
        public static final double red = 0.61; //resevered for errors/ warnings
        public static final double redOrange = 0.63;
        public static final double orange = 0.65;
        public static final double gold = 0.67;
        public static final double yellow = 0.69;
        public static final double lawnGreen = 0.71;
        public static final double lime = 0.73;
        public static final double darkGreen = 0.75;
        public static final double green = 0.77;
        public static final double blueGreen = 0.79;
        public static final double aqua = 0.81;
        public static final double skyBlue = 0.83;
        public static final double darkBlue = 0.85;
        public static final double blue = 0.87;
        public static final double blueViolet = 0.89;
        public static final double violet = 0.91;
        public static final double white = 0.93;
        public static final double gray = 0.95;
        public static final double darkGray = 0.97;

        

        public static final double rainbow_rainbow = -0.99; //rainbow pattern and rainbow colours

        public static final double off = 0.99;

        //pride flag presets
        public static final double[] trans_flag = {
        1.0, darkGreen, green, lime, blueGreen, lawnGreen, violet, blueViolet
        };


    }
}
