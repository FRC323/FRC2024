package frc.robot.subsystems.led;

//color chart:
//https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf (pg 14-17)
public enum LedColor {
    Yellow(0.69),
    Red(0.61),
    Red_Blink(-0.25),
    Red_Chase(-0.31),
    Orange(0.65),
    Green(0.77),
    Blue(0.87),
    Blue_Blink(-0.23),
    Blue_Chase(-0.29),
    White(0.93),
    RAINBOW_PARTY(-0.97),
    Black(0.99);

    private final double color;

    LedColor(double color) {
        this.color = color;
    }

    public double get() {
        return this.color;
    }
}
