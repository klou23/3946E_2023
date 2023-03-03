import javax.swing.JFrame;
import javax.swing.JPanel;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.StringTokenizer;

public class Visualizer extends JPanel {

    /**
     * Tuning trials
     * 0: 2, 0.7
     * 1: 10, 0.4
     * 2: 5, 0
     * 3: 10, 0
     * 4: 5, 0.4
     * 5: 5, 0.7
     */

    final static int logNum = 4;

    final static boolean showTarget = true;
    final static boolean showActual = true;
    final static boolean autoPlay = true;
    final static double metersShown = 4;

    static int frame = 0;

    static List<RobotState> targetStates = new ArrayList<>();
    static List<RobotState> actualStates = new ArrayList<>();

    final static Grid grid = new Grid(1.8288, 3);

    final static String pathName = "DriveTest";

    final static double[][] transform = {{500/metersShown,0,500},{0,-500/metersShown,500},{0,0,1}};

    JFrame jfrm;

    void readTarget() throws IOException{
        BufferedReader br = new BufferedReader(new FileReader("/Volumes/RECORDER/log_target" + logNum + ".txt"));

        String line;
        while((line = br.readLine()) != null){
            StringTokenizer st = new StringTokenizer(line);
            double x = Double.parseDouble(st.nextToken());
            double y = Double.parseDouble(st.nextToken());
            double theta = Double.parseDouble(st.nextToken());
            targetStates.add(new RobotState(x, y, theta));
        }
    }

    void readActual() throws IOException{
        BufferedReader br = new BufferedReader(new FileReader("/Volumes/RECORDER/log_actual" + logNum + ".txt"));

        String line;
        while((line = br.readLine()) != null){
            StringTokenizer st = new StringTokenizer(line);
            double x = Double.parseDouble(st.nextToken());
            double y = Double.parseDouble(st.nextToken());
            double theta = Double.parseDouble(st.nextToken());
            actualStates.add(new RobotState(x, y, theta));
        }
    }

    public Visualizer() throws IOException{
        if(showTarget) readTarget();
        if(showActual) readActual();

        jfrm = new JFrame();
        jfrm.setSize(1000, 1000);
        jfrm.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jfrm.setResizable(false);
        jfrm.add(this);
        jfrm.setVisible(true);

    }

    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2d.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);

        grid.draw(g2d, metersShown, transform);

        for(RobotState target : targetStates){
            target.drawPoint(g2d, transform, Color.BLACK);
        }
        for(RobotState actual : actualStates){
            actual.drawPoint(g2d, transform, Color.RED);
        }
    }

    public static void main(String[] args) throws IOException {
        Visualizer v = new Visualizer();
    }

}
