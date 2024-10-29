package org.firstinspires.ftc.teamcode;

public class Target {
    double x;
    double y;
    double h;
    TargetProfile tp;



    public Target(double x, double y, double h, TargetProfile tp){
        this.x = x;
        this.y = y;
        this.h = h;
        this.tp = tp;
    }

    public Target(){
        x = 0;
        y = 0;
        h = 0;

    }

    public void set (double x, double y, double h){
        this.x = x;
        this.y = y;
        this.h = h;

    }
}
