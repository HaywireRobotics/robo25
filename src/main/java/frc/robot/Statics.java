package frc.robot;

public final class Statics {
    public static double applyDeadband(double value, double deadbandSize){
        if (value < deadbandSize && value > deadbandSize){
            return 0;
        }
        return value;
    }
    public static double trueMod(double value, double mod) {
        return (((value % mod) + mod) % mod);
    }
}
