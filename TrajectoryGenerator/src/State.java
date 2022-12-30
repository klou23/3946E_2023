/**
 * File: State.java
 * Author: Kevin Lou
 * Created On: Dec 29 2022
 * Last Updated: Dec 29 2022
 *
 * Copyright (c) 2022 3946E
 *
 * Summary of file:
 *  - Trajectory state. Includes: x, y, theta, v, omega
 */

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Polygon;

public class State {
    double x;
    double y;
    double theta;
    double v;
    double omega;

    public State(double x, double y, double theta, double v, double omega) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.v = v;
        this.omega = omega;
    }

    public void drawPoint(Graphics2D g, double[][] transform){

        int x0 = (int) Math.round(x*transform[0][0] + y*transform[0][1] + transform[0][2]);
        int y0 = (int) Math.round(x*transform[1][0] + y*transform[1][1] + transform[1][2]);

        g.setColor(Color.BLUE);
//        g.fillOval(x0-1, y0-1, 2, 2);
        g.setStroke(new BasicStroke(1));
        g.drawLine(x0,y0,x0,y0);
    }

    public void drawRobot(Graphics2D g, double[][] transform){

        int x0 = (int) Math.round(x*transform[0][0] + y*transform[0][1] + transform[0][2]);
        int y0 = (int) Math.round(x*transform[1][0] + y*transform[1][1] + transform[1][2]);

        Polygon p = new Polygon();
        p.addPoint((int)(15*Math.cos(theta)), (int)(-15*Math.sin(theta)));
        p.addPoint((int)(15*Math.cos(theta+0.75*Math.PI)), (int)(-15*Math.sin(theta+0.75*Math.PI)));
        p.addPoint((int)(7*Math.cos(theta-Math.PI)), (int)(-7*Math.sin(theta-Math.PI)));
        p.addPoint((int)(15*Math.cos(theta-0.75*Math.PI)), (int)(-15*Math.sin(theta-0.75*Math.PI)));

        p.translate(x0, y0);

        g.setColor(Color.GREEN);
        g.fillPolygon(p);

        g.setStroke(new BasicStroke(2));
        g.setColor(Color.BLACK);
        g.drawPolygon(p);
    }
}
