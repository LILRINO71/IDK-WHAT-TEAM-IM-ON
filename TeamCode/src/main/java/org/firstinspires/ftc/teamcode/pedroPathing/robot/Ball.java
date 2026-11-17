package org.firstinspires.ftc.teamcode.pedroPathing.robot;

public enum Ball {
    NONE, POLLEN, RED_NECTAR, BLUE_NECTAR;

    public boolean isNectar() {
        return this == RED_NECTAR || this == BLUE_NECTAR;
    }

    public double grams() {
        if (this == POLLEN) return RobotConfig.POLLEN_G;
        if (isNectar()) return RobotConfig.NECTAR_G;
        return 0;
    }

    public boolean ours(Alliance alliance) {
        if (this == POLLEN) return true;
        return alliance == Alliance.RED ? this == RED_NECTAR : this == BLUE_NECTAR;
    }

    public String shortName() {
        switch (this) {
            case POLLEN: return "P";
            case RED_NECTAR: return "Nr";
            case BLUE_NECTAR: return "Nb";
            default: return "-";
        }
    }
}
