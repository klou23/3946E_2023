/**
 * File: BezierStartPoint.java
 * Author: Kevin Lou
 * Created On: Dec 23 2022
 * Last Updated: Dec 23 2022
 *
 * Copyright (c) 2022 3946E
 *
 * Summary of file:
 *  - Representation of the starting point in a bezier path
 */

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;

public class BezierStartPoint {

    double x;
    double y;
    double theta;
    double distance;

    public BezierStartPoint(double x, double y, double theta, double distance) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.distance = distance;
    }

    public Point getStartPoint(){
        return new Point(x,y);
    }

    public Point getControlPoint(){
        return new Point(x + distance*Math.cos(theta), y + distance*Math.sin(theta));
    }

    public void draw(Graphics2D g, double[][] transform){
        int x0 = (int) Math.round(x*transform[0][0] + y*transform[0][1] + transform[0][2]);
        int y0 = (int) Math.round(x*transform[1][0] + y*transform[1][1] + transform[1][2]);

        double controlX = getControlPoint().x;
        double controlY = getControlPoint().y;
        int x1 = (int) Math.round(controlX*transform[0][0] + controlY*transform[0][1] + transform[0][2]);
        int y1 = (int) Math.round(controlX*transform[1][0] + controlY*transform[1][1] + transform[1][2]);

        g.setColor(Color.MAGENTA);
        g.setStroke(new BasicStroke(3));
        g.drawLine(x0, y0, x1, y1);
        g.fillOval(x1-4, y1-4, 8, 8);

        g.setColor(Color.BLACK);
        g.fillOval(x0-6, y0-6, 12, 12);
    }

}
