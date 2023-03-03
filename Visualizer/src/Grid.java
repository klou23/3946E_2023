/**
 * File: Grid.java
 * Author: Kevin Lou
 * Created On: Dec 29 2022
 * Last Updated: Dec 29 2022
 *
 * Copyright (c) 2022 3946E
 *
 * Summary of file:
 *  - Used to draw cartesian plane
 */

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;

public class Grid {

    double bigGrid;
    double smallGrid;

    public Grid(double bigGrid, double divisions) {
        this.bigGrid = bigGrid;
        this.smallGrid = bigGrid/divisions;
    }

    public void draw(Graphics2D g, double metersShown, double[][] transform){
        for(double i = 0; i <= metersShown; i += smallGrid){
            double x0 = i;
            double y0 = -metersShown;
            double x1 = i;
            double y1 = metersShown;

            double x2 = -metersShown;
            double y2 = i;
            double x3 = metersShown;
            double y3 = i;

            if(Math.abs(i) < 0.0001) g.setColor(Color.BLACK);
            else if(Math.abs(i)%bigGrid < 0.0001 || Math.abs(i)%bigGrid > bigGrid-0.0001) g.setColor(Color.GRAY);
            else g.setColor(Color.LIGHT_GRAY);
            g.setStroke(new BasicStroke(1));

            int x0_t = (int) Math.round(x0*transform[0][0] + y0*transform[0][1] + transform[0][2]);
            int y0_t = (int) Math.round(x0*transform[1][0] + y0*transform[1][1] + transform[1][2]);
            int x1_t = (int) Math.round(x1*transform[0][0] + y1*transform[0][1] + transform[0][2]);
            int y1_t = (int) Math.round(x1*transform[1][0] + y1*transform[1][1] + transform[1][2]);

            g.drawLine(x0_t, y0_t, x1_t, y1_t);

            int x2_t = (int) Math.round(x2*transform[0][0] + y2*transform[0][1] + transform[0][2]);
            int y2_t = (int) Math.round(x2*transform[1][0] + y2*transform[1][1] + transform[1][2]);
            int x3_t = (int) Math.round(x3*transform[0][0] + y3*transform[0][1] + transform[0][2]);
            int y3_t = (int) Math.round(x3*transform[1][0] + y3*transform[1][1] + transform[1][2]);

            g.drawLine(x2_t, y2_t, x3_t, y3_t);
        }

        for(double i = -smallGrid; i >= -metersShown; i -= smallGrid){
            double x0 = i;
            double y0 = -metersShown;
            double x1 = i;
            double y1 = metersShown;

            double x2 = -metersShown;
            double y2 = i;
            double x3 = metersShown;
            double y3 = i;

            if(Math.abs(i) < 0.0001) g.setColor(Color.BLACK);
            else if(Math.abs(i)%bigGrid < 0.0001 || Math.abs(i)%bigGrid > bigGrid-0.0001) g.setColor(Color.GRAY);
            else g.setColor(Color.LIGHT_GRAY);
            g.setStroke(new BasicStroke(1));

            int x0_t = (int) Math.round(x0*transform[0][0] + y0*transform[0][1] + transform[0][2]);
            int y0_t = (int) Math.round(x0*transform[1][0] + y0*transform[1][1] + transform[1][2]);
            int x1_t = (int) Math.round(x1*transform[0][0] + y1*transform[0][1] + transform[0][2]);
            int y1_t = (int) Math.round(x1*transform[1][0] + y1*transform[1][1] + transform[1][2]);

            g.drawLine(x0_t, y0_t, x1_t, y1_t);

            int x2_t = (int) Math.round(x2*transform[0][0] + y2*transform[0][1] + transform[0][2]);
            int y2_t = (int) Math.round(x2*transform[1][0] + y2*transform[1][1] + transform[1][2]);
            int x3_t = (int) Math.round(x3*transform[0][0] + y3*transform[0][1] + transform[0][2]);
            int y3_t = (int) Math.round(x3*transform[1][0] + y3*transform[1][1] + transform[1][2]);

            g.drawLine(x2_t, y2_t, x3_t, y3_t);
        }
    }
}
