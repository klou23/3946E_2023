/**
 * File: Visualizer.java
 * Author: Kevin Lou
 * Created On: Dec 27 2022
 * Last Updated: Dec 29 2022
 *
 * Copyright (c) 2022 3946E
 *
 * Summary of file:
 *  - Allows for visualization of trajectories
 */

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.Timer;
import java.awt.BasicStroke;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.StringTokenizer;

public class Visualizer extends JPanel implements ActionListener {

    final static boolean showInput = false;
    final static boolean showOutput = true;
    final static boolean showOutputRobot = true;
    final static boolean showActual = false;
    final static boolean showActualRobot = false;

    final static double metersShown = 1.25;

    final static Grid grid = new Grid(1.8288, 3);

    final static String pathName = "Trajectory1";

    final static double[][] transform = {{500/metersShown,0,500},{0,-500/metersShown,500},{0,0,1}};

    Timer t = new Timer(5, this);

    JFrame jfrm;

    List<BezierWaypoint> waypoints = new ArrayList<>();
    List<BezierStartPoint> startPoints = new ArrayList<>();
    List<BezierEndPoint> endPoints = new ArrayList<>();
    List<State> trajectory = new ArrayList<>();
    List<RobotState> robot = new ArrayList<>();

    public Visualizer() throws IOException{
        read();

        jfrm = new JFrame();
        jfrm.setSize(1000, 1000);
        jfrm.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jfrm.setResizable(false);
        jfrm.add(this);
        jfrm.setVisible(true);

        t.start();
    }

    /**
     * Reads in the input
     * @throws IOException
     */
    public void read() throws IOException{
        BufferedReader br;
        if(showInput){
            br = new BufferedReader(new FileReader("Input/Path" + pathName + ".txt"));
            String line;
            while((line = br.readLine()) != null){
                if(line.equals("DRIVE_PATH")){
                    StringTokenizer st = new StringTokenizer(br.readLine());
                    st.nextToken(); st.nextToken(); st.nextToken();
                    int waypointCnt = Integer.parseInt(st.nextToken());
                    st = new StringTokenizer(br.readLine());
                    startPoints.add(new BezierStartPoint(Double.parseDouble(st.nextToken()),
                            Double.parseDouble(st.nextToken()),
                            Double.parseDouble(st.nextToken()),
                            Double.parseDouble(st.nextToken())));
                    for(int i = 0; i < waypointCnt; i++){
                        st = new StringTokenizer(br.readLine());
                        BezierWaypoint waypoint = new BezierWaypoint(Double.parseDouble(st.nextToken()),
                                Double.parseDouble(st.nextToken()),
                                Double.parseDouble(st.nextToken()),
                                Double.parseDouble(st.nextToken()),
                                Double.parseDouble(st.nextToken()));
                        waypoints.add(waypoint);
                    }
                    st = new StringTokenizer(br.readLine());
                    endPoints.add(new BezierEndPoint(Double.parseDouble(st.nextToken()),
                            Double.parseDouble(st.nextToken()),
                            Double.parseDouble(st.nextToken()),
                            Double.parseDouble(st.nextToken())));
                }
            }
        }

        if(showOutput || showOutputRobot){
            br = new BufferedReader(new FileReader("Output/Path" + pathName + ".txt"));
            int n = Integer.parseInt(br.readLine());
            for(int i = 0; i < n; i++){
                StringTokenizer st = new StringTokenizer(br.readLine(),",");
                double x = Double.parseDouble(st.nextToken()) * 0.0254;
                double y = Double.parseDouble(st.nextToken()) * 0.0254;
                double theta = Double.parseDouble(st.nextToken()) * Math.PI / 180.0;
                double v = Double.parseDouble(st.nextToken());
                double omega = Double.parseDouble(st.nextToken());
                trajectory.add(new State(x, y, theta, v, omega));
            }
        }

        if(showActual || showActualRobot){
            br = new BufferedReader(new FileReader("/Volumes/RECORDER/log.txt"));
            String line;
            while((line = br.readLine()) != null){
                StringTokenizer st = new StringTokenizer(line);
                double x = Double.parseDouble(st.nextToken());
                double y = Double.parseDouble(st.nextToken());
                double theta = Double.parseDouble(st.nextToken());
                robot.add(new RobotState(x, y, theta));
            }
        }
    }

    public void paintComponent(Graphics g){
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2d.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);

        grid.draw(g2d, metersShown, transform);

        if(showInput){
            for(BezierStartPoint b : startPoints){
                b.draw(g2d, transform);
            }
            for(BezierWaypoint b : waypoints){
                b.draw(g2d, transform);
            }
            for(BezierEndPoint b : endPoints){
                b.draw(g2d, transform);
            }
        }

        if(showOutput){
            for(State s : trajectory){
                s.drawPoint(g2d, transform);
            }
        }

        if(showOutputRobot){
            trajectory.get(Math.min(index1, trajectory.size()-1)).drawRobot(g2d, transform);
        }

        if(showActual){
            for(RobotState s : robot){
                s.drawPoint(g2d, transform);
            }
        }

        if(showActualRobot){
            robot.get(Math.min(index2, trajectory.size()-1)).drawRobot(g2d, transform);
        }
    }

    int index1 = 0;
    int index2 = 0;
    public void actionPerformed(ActionEvent e){
        index1++;
        index2++;
        if(index1 >= trajectory.size() + 200) index1 = 0;
        if(index2 >= robot.size() + 200) index2 = 0;
        repaint();
    }

    public static void main(String[] args) throws IOException{
        new Visualizer();
    }
}
