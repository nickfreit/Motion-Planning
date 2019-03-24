import java.util.*;

float SCALE = 20;
int SAMPLE_SIZE = 100;
boolean PLANNING = true;
boolean PAUSED = true;

Object AGENT;
Object OBSTACLE;
Object GOAL;

PVector AGENT_INIT_POS = new PVector(-9, -9);;

ArrayList<Point> PATH;

ArrayList<Point> PTS;


void setup() {
  size(960, 960, P3D);
  AGENT = new Object(AGENT_INIT_POS.x, AGENT_INIT_POS.y, 0, 0, 0.5, SCALE);
  AGENT_INIT_POS = new PVector(AGENT.pos.x, AGENT.pos.y);
  OBSTACLE = new Object(0, 0, 0, 0, 2 , SCALE);
  OBSTACLE.configRadius = OBSTACLE.radius + AGENT.radius;
  GOAL = new Object(9, 9, 0, 0, 1, SCALE);
  PTS = new ArrayList();
  
  PTS.add(new Point(AGENT_INIT_POS));
  PTS.add(new Point(GOAL.pos));
  PATH = new ArrayList();
}

void draw() {
  background(200);
  
  AGENT.display();
  OBSTACLE.display();
  GOAL.display();
  
  for (int i = 0; i < PTS.size(); i++) {
    stroke(0, 50);
    strokeWeight(1);
    point(PTS.get(i).pos.x, PTS.get(i).pos.y);
  }
  for (int i = 0; i < PTS.size(); i++) {
    Point p = PTS.get(i);
    for (int j = 0; j < p.neighbors.size(); j++) {
      Point q = p.neighbors.get(j);
      line(p.pos.x, p.pos.y, q.pos.x, q.pos.y);
    }
  }
  
  if (PLANNING && !PAUSED) { 
    samplePoints(PTS, SAMPLE_SIZE, OBSTACLE);
    connectPoints(PTS, OBSTACLE);
    PLANNING = !uniformCostSearch(PTS, PATH);
  } else {
    for (int i = 0; i < PATH.size(); i++) {
      stroke(0, 255, 0);
      strokeWeight(2);
      if (i > 1) {
        line(PATH.get(i-1).pos.x, PATH.get(i-1).pos.y, PATH.get(i).pos.x, PATH.get(i).pos.y); 
      } else {
        line(AGENT_INIT_POS.x, AGENT_INIT_POS.y, PATH.get(i).pos.x, PATH.get(i).pos.y);
      }
    }
    Point a = new Point(AGENT.pos);
    int i = PATH.size() - 1;
    while(i > 0 && !canConnectPoints(a, PATH.get(i), OBSTACLE)) {
      i--;
    }
    if (i > 0) {
      PVector vDir = new PVector(PATH.get(i).pos.x - AGENT.pos.x, PATH.get(i).pos.y - AGENT.pos.y);
      vDir.normalize();
      vDir.mult(100);
      AGENT.vel = vDir;
      AGENT.update(.01);
    }
  }
  
  
  PAUSED = true;
}

class Object {
  PVector pos;
  PVector vel;
  float radius;
  float configRadius;
  float scale;
  
  Object(float x, float y, float vx, float vy, float r, float s) {
    scale = s;
    pos = new PVector(((x + scale / 2) / scale) * width, ((-y + scale / 2) / scale) * height);
    vel = new PVector(((vx + scale / 2) / scale) * width, ((-vy + scale / 2) / scale) * height);
    radius = r * (width / scale);
    configRadius = 0;
  }
  
  void update(float dt) {
    pos.add(PVector.mult(vel, dt));
  }
  
  void display() {
    fill(0, 0, 255);
    noStroke();
    ellipse(pos.x, pos.y, 2*radius, 2*radius);
  }
}

class Point {
  PVector pos;
  ArrayList<Point> neighbors;
  boolean visited = false;
  
  Point(PVector p) {
    pos = p;
    neighbors = new ArrayList();
  }
  
  Point() {
    pos = new PVector(random(0, width), random(0, height));
    neighbors = new ArrayList();
  }
}

void samplePoints(ArrayList<Point> samples, int n, Object obstacle) {
  for (int i = 0; i < n; i++) {
    float xDist, yDist;
    Point p;
    do {
      p = new Point();
      xDist = p.pos.x - obstacle.pos.x;
      yDist = p.pos.y - obstacle.pos.y;
    } while (xDist * xDist + yDist * yDist < obstacle.configRadius * obstacle.configRadius);
    samples.add(p);
  }
}


void connectPoints(ArrayList<Point> samples, Object obstacle) {
  for (int i = 0; i < samples.size(); i++) {
    Point p = samples.get(i);
    for (int j = 0; j < samples.size(); j++) {
      if (j != i && !p.neighbors.contains(samples.get(j)) && canConnectPoints(p, samples.get(j), obstacle)) {
        p.neighbors.add(samples.get(j));
      }
    }
  }
}


boolean canConnectPoints(Point a, Point b, Object obstacle) {
  PVector segment = PVector.sub(a.pos, b.pos);
  PVector toObstacle = PVector.sub(a.pos, obstacle.pos);
  float A = segment.dot(segment);
  float B = 2 * toObstacle.dot(segment);
  float C = toObstacle.dot(toObstacle) - obstacle.configRadius * obstacle.configRadius;
  return (B * B - 4 * A * C) < 0 && PVector.dist(a.pos, b.pos) < 1000;
}


boolean breadthFirstSearch(ArrayList<Point> points, ArrayList<Point> path) {
  Queue<ArrayList<Point>> q = new LinkedList<ArrayList<Point>>();
  Point start = points.get(0);
  Point goal = points.get(1);
  
  start.visited = true;
  ArrayList<Point> starting = new ArrayList<Point>();
  starting.add(start);
  q.add(starting);
  
  while (q.size() > 0) {
    ArrayList<Point> p = q.remove();
    Point m = p.get(p.size() - 1);
    if (m == goal) {
      for (int i = 0; i < p.size(); i++) {
        path.add(i, p.get(i));
      }
      return true;
    }
    for(int i = 0; i < m.neighbors.size(); i++) {
      Point n = m.neighbors.get(i);
      if (!n.visited) {
        ArrayList<Point> newPath = new ArrayList<Point>();
        for (int j = 0; j < p.size(); j++) {
          newPath.add(p.get(j));
        }
        newPath.add(n);
        n.visited = true;
        q.add(newPath);
      }
    }
  }
  return false;
}

boolean uniformCostSearch(ArrayList<Point> points, ArrayList<Point> path) {

  Point start = points.get(0);
  Point goal = points.get(1);
  
  Comparator<ArrayList<Point>> c = new CostComparator();
  PriorityQueue<ArrayList<Point>> q = new PriorityQueue<ArrayList<Point>>(11, c);
  
  start.visited = true;
  ArrayList<Point> starting = new ArrayList<Point>();
  starting.add(start);
  q.add(starting);
  
  while (q.size() > 0) {
    ArrayList<Point> p = q.poll();
    //printPri(p);
    
    Point m = p.get(p.size() - 1);
    if (m == goal) {
      for (int i = 0; i < p.size(); i++) {
        path.add(i, p.get(i));
      }
      return true;
    }
    for(int i = 0; i < m.neighbors.size(); i++) {
      Point n = m.neighbors.get(i);
      ArrayList<Point> newPath = new ArrayList<Point>();
      for (int j = 0; j < p.size(); j++) {
        newPath.add(p.get(j));
      }
      newPath.add(n);
      if (!n.visited) {
        n.visited = true;
        q.add(newPath);
      } else {
        ArrayList<Point> toRemove = null;
        for (ArrayList<Point> el : q) {
          if (el.get(el.size() - 1) == n && calcPriority(el) > calcPriority(newPath)) {
            toRemove = el;
          }
        }
        if (toRemove != null) {
          q.remove(toRemove);
          q.add(newPath);
        }
      }
    }
  }
  return false;
}


float calcPriority(ArrayList<Point> p) {
  float sum = 0;
  for (int i = 0; i < p.size() - 1; i++) {sum += PVector.dist(p.get(i).pos, p.get(i+1).pos); }
  return sum;
}
void printPri(ArrayList<Point> p) {
  float sum = 0;
  for (int i = 0; i < p.size() - 1; i++) {sum += PVector.dist(p.get(i).pos, p.get(i+1).pos); }
  println(sum);
}

public class CostComparator implements Comparator<ArrayList<Point>>
{
  public int compare(ArrayList<Point> a, ArrayList<Point> b) {
    float asum = 0;
    float bsum = 0;
    for (int i = 0; i < a.size()-1; i++) { asum += PVector.dist(a.get(i).pos, a.get(i+1).pos); }
    for (int i = 0; i < b.size()-1; i++) { bsum += PVector.dist(b.get(i).pos, b.get(i+1).pos); }
    if (asum > bsum) {
      return 1;
    } else if (asum < bsum) {
      return -1;
    } else {
      return 0;
    }
  }
}

void keyPressed() {
  PAUSED = !PAUSED;
}
