import java.util.*;

// --------------- GLOBAL VARS --------------- 
int NUM_AGENTS = 25;
int NUM_OBSTACLES = 25;
float OBSTACLE_RADIUS = 20;
float AGENT_RADIUS = 5;
float GOAL_RADIUS = 20;
float AGENT_X = 960.0 / 10;
float AGENT_Y = 9 * 960.0 / 10;
float GOAL_X = 960.0 / 10;
float GOAL_Y = 960.0 / 10;
int SAMPLE_SIZE = 400;

boolean PLANNING = false;
boolean PLACE = true;

Color OBSTACLE_COLOR = new Color(255, 0, 0);
Color AGENT_COLOR = new Color(0, 255, 0);
Color GOAL_COLOR = new Color(0, 0, 255);

ArrayList<Agent> AGENTS = new ArrayList<Agent>(NUM_AGENTS);;
ArrayList<Obstacle> OBSTACLES = new ArrayList<Obstacle>(NUM_OBSTACLES);
ArrayList<Obstacle> GOALS = new ArrayList<Obstacle>(NUM_AGENTS);
ArrayList<Node> GRAPH = new ArrayList<Node>();


// --------------- SETUP --------------- 
void setup() {
  size(960, 960, P3D);
  
  for (int i = 0; i < NUM_OBSTACLES; i++) {
    OBSTACLES.add(i, new Obstacle(random(0, width), random(0, height), OBSTACLE_RADIUS, OBSTACLE_RADIUS+AGENT_RADIUS, OBSTACLE_COLOR));
    while(OBSTACLES.get(i).isCollidingWith(OBSTACLES)) {
      OBSTACLES.set(i, new Obstacle(random(0, width), random(0, height), OBSTACLE_RADIUS, OBSTACLE_RADIUS+AGENT_RADIUS, OBSTACLE_COLOR));
    }
  }
  
  for (int i = 0; i < NUM_AGENTS; i++) {
    AGENT_COLOR = new Color(0, 0, 255);
    GOAL_COLOR = AGENT_COLOR;
    AGENTS.add(i, new Agent(width-AGENT_X + random(-50, 50), AGENT_Y  + random(-50, 50), 0, 0, AGENT_RADIUS, AGENT_COLOR));
    GOALS.add(i, new Obstacle(GOAL_X, GOAL_Y, GOAL_RADIUS, 0, GOAL_COLOR));
    while(AGENTS.get(i).isCollidingWith(OBSTACLES)) {
      AGENTS.set(i, new Agent(width-AGENT_X + random(-50, 50), AGENT_Y  + random(-50, 50), 0, 0, AGENT_RADIUS, AGENT_COLOR));
      GOALS.set(i, new Obstacle(GOAL_X, GOAL_Y, GOAL_RADIUS, 0, GOAL_COLOR));
    }
  }
}


// --------------- DRAW --------------- 
void draw() {
  background(200);
  
  if (mousePressed && PLACE) {
    OBSTACLES.add(new Obstacle(mouseX, mouseY, OBSTACLE_RADIUS, OBSTACLE_RADIUS + AGENT_RADIUS, OBSTACLE_COLOR));
  }
  
  if (keyPressed && key == 'a' && PLACE) {
    AGENTS.add(new Agent(mouseX, mouseY, 0, 0, AGENT_RADIUS, AGENT_COLOR));
    GOALS.add(new Obstacle(GOAL_X, GOAL_Y, GOAL_RADIUS, 0, GOAL_COLOR));
  }
  
  if (keyPressed && key == 'f' && PLACE) {
    GOAL_X = mouseX;
    GOAL_Y = mouseY;
    float r = random(0, 255);
    float g = random(0, 255);
    float b = random(0, 255);
    GOAL_COLOR = new Color(r, g, b);
    AGENT_COLOR = new Color(r, g, b);
  }
 
  if (PLANNING) {
    generatePRM(GRAPH, OBSTACLES, SAMPLE_SIZE);
    for (int i = 0; i < AGENTS.size(); i++) {
      ArrayList<Node> searchGraph = new ArrayList<Node>();
      setupGraph(GRAPH, searchGraph, AGENTS.get(i), GOALS.get(i), OBSTACLES);
      aStarSearch(searchGraph, AGENTS.get(i));
      PLANNING = false;
    }
  } else if (!PLANNING && !PLACE) {
    for (int i = 0; i < AGENTS.size(); i++) {
      if (!AGENTS.get(i).atGoal) {
        AGENTS.get(i).checkGoal(GOALS.get(i));
        AGENTS.get(i).update(.01, OBSTACLES, AGENTS);
      }
    }
  }
  
  for (Agent agent : AGENTS) {
    if (!agent.atGoal) {
      agent.display();
      agent.displayPath();
    }
  }
  for (Obstacle ob : OBSTACLES) {
    ob.display();
  }
  for (Obstacle goal : GOALS) {  
    goal.display();
  }
}


// --------------- PATH PLANNING --------------- 
void setupGraph(ArrayList<Node> graph, ArrayList<Node> searchGraph, Agent agent, Obstacle goal, ArrayList<Obstacle> obs) {
  searchGraph.add(0, new Node(agent.pos.x, agent.pos.y));
  searchGraph.add(1, new Node(goal.pos.x, goal.pos.y));
  for (int i = 0; i < graph.size(); i++) {
    Node nbr = graph.get(i);
    float dist = PVector.dist(nbr.pos, agent.pos);
    if (!searchGraph.get(0).neighbors.contains(nbr) && !intersectsObstacle(obs, searchGraph.get(0).pos, nbr.pos) && dist < 200) {
      searchGraph.get(0).neighbors.add(nbr);
      nbr.neighbors.add(searchGraph.get(0));
    }
  }
  for (int i = 0; i < graph.size(); i++) {
    Node nbr = graph.get(i);
    float dist = PVector.dist(nbr.pos, goal.pos);
    if (!searchGraph.get(1).neighbors.contains(nbr) && !intersectsObstacle(obs, searchGraph.get(1).pos, nbr.pos) && dist < 200) {
      searchGraph.get(1).neighbors.add(nbr);
      nbr.neighbors.add(searchGraph.get(1));
    }
  }
  for (int i = 0; i < graph.size(); i++) {
    graph.get(i).visited = false;
    searchGraph.add(i+2, graph.get(i));
  }
}

public class CostComparator implements Comparator<Path>
{
  public int compare(Path a, Path b) {
    if (a.fscore > b.fscore) {
      return 1;
    } else if (b.fscore > a.fscore) {
      return -1;
    } else {
      return 0;
    }
  }
}

boolean aStarSearch(ArrayList<Node> graph, Agent agent) {

  Node start = graph.get(0);
  Node goal = graph.get(1);
  
  Comparator<Path> c = new CostComparator();
  PriorityQueue<Path> q = new PriorityQueue<Path>(11, c);
  
  start.visited = true;
  Path starting = new Path();
  starting.path.add(start);
  starting.fscore = PVector.dist(starting.path.get(starting.path.size()-1).pos, goal.pos);
  q.add(starting);
 
  while (q.size() > 0) {
    Path p = q.poll();
    
    Node m = p.path.get(p.path.size() - 1);
    if (m == goal) {
      for (int i = 0; i < p.path.size(); i++) {
        agent.path.add(i, p.path.get(i));
      }
      return true;
    }
    for(int i = 0; i < m.neighbors.size(); i++) {
      Node n = m.neighbors.get(i);
      Path newPath = new Path();
      for (int j = 0; j < p.path.size(); j++) {
        newPath.path.add(p.path.get(j));
      }
      newPath.path.add(n);
      for (int j = 0; j < newPath.path.size()-1; j++) {
        newPath.fscore += PVector.dist(newPath.path.get(j).pos, newPath.path.get(j+1).pos);
      }
      newPath.fscore += PVector.dist(newPath.path.get(newPath.path.size()-1).pos, goal.pos);
      if (!n.visited) {
        n.visited = true;
        q.add(newPath);
      } else {
        Path toRemove = null;
        for (Path el : q) {
          if (el.path.get(el.path.size() - 1) == n && el.fscore > newPath.fscore) {
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

// --------------- PRM --------------- 
void displayPRM(ArrayList<Node> graph) {
  for (int i = 0; i < graph.size(); i++) {
    stroke(0, 5);
    strokeWeight(1);
    Node node = graph.get(i);
    for (int j = 0; j < node.neighbors.size(); j++) {
      Node nbr = node.neighbors.get(j);
      line(node.pos.x, node.pos.y, nbr.pos.x, nbr.pos.y);
    }
  }
}

void generatePRM(ArrayList<Node> graph, ArrayList<Obstacle> obs, int n) {
  for (int i = 0; i < n; i++) {
    float xNode = random(0, width);
    float yNode = random(0, height);
    while(inObstacle(obs, xNode, yNode)) {
      xNode = random(0, width);
      yNode = random(0, height);
    }
    graph.add(new Node(xNode, yNode));
  }
  
  for (int i = 0; i < n; i++) {
    Node node = graph.get(i);
    for (int j = 0; j < n; j++) {
      Node nbr = graph.get(j);
      float dist = PVector.dist(node.pos, nbr.pos);
      if (j != i && !node.neighbors.contains(nbr) && !intersectsObstacle(obs, node.pos, nbr.pos) && dist < 200) {
        node.neighbors.add(nbr);
      }
    }
  }
}


// --------------- COLLISION CHECKING --------------- 
boolean inObstacle(ArrayList<Obstacle> obs, float x, float y) {
  for (Obstacle ob : obs) {
    if (ob.isInside(x, y)) {
      return true;
    }
  }
  return false;
}

boolean intersectsObstacle(ArrayList<Obstacle> obs, PVector p1, PVector p2) {
  for (Obstacle ob: obs) {
    if (ob.intersect(p1, p2)) {
      return true;
    }
  }
  return false;
}


// --------------- AGENT CLASS --------------- 
class Agent {
  PVector pos;
  PVector init_pos;
  PVector vel;
  PVector acc;
  float radius;
  Color col;
  ArrayList<Node> path;
  
  float maxSpeed = 200;
  float separation = 25;
  float neighborhood = 100;
  float obstacleBuffer = 5;
  boolean atGoal = false;
  float goalBuffer = 20;
  
  Agent(float x, float y, float vx, float vy, float r, Color c) {
    pos = new PVector(x, y);
    vel = new PVector(vx, vy);
    acc = new PVector(0, 0);
    radius = r;
    col = c;
    path = new ArrayList<Node>();
  }
  
  void checkGoal(Obstacle goal) {
    float dist = PVector.dist(goal.pos, pos);
    if (dist < goalBuffer) {
      atGoal = true;
    }
  }
  
  void update(float dt, ArrayList<Obstacle> obs, ArrayList<Agent> agents) {
    PVector pForce = pathForce(obs);
    PVector sForce = separationForce(agents);
    PVector cForce = cohesionForce(agents);
    PVector aForce = alignForce(agents);
    PVector oForce = obstacleForce(obs);
    
    acc.add(pForce.mult(7));
    acc.add(sForce.mult(7));
    acc.add(cForce.mult(1));
    acc.add(aForce.mult(1));
    acc.add(oForce.mult(5));
    
    vel.add(PVector.mult(acc, dt));  
    vel.limit(maxSpeed);
    pos.add(PVector.mult(vel, dt));
    acc.set(0, 0);
  }
  
  PVector pathForce(ArrayList<Obstacle> obs) {
    PVector pForce = new PVector(0, 0);
    int i = path.size() - 1;
    while (i > 0 && intersectsObstacle(obs, pos, path.get(i).pos)) {
      i--;
    }
    if (i > 0) {
      pForce.set(path.get(i).pos.x - pos.x, path.get(i).pos.y - pos.y);
      pForce.normalize();
      pForce.mult(maxSpeed);
      pForce.sub(vel);
    }
    return pForce;
  }
  
  PVector separationForce(ArrayList<Agent> agents) {
    PVector sForce = new PVector(0, 0);
    for (Agent agent: agents) {
      if (agent.atGoal) {
        continue;
      }
      PVector dif = PVector.sub(pos, agent.pos);
      float mag = dif.mag();
      if (mag > 0 && mag < separation) {
        dif.normalize();
        dif.mult(1.0 / mag);
        sForce.add(dif);
      }
    }
    sForce.normalize();
    sForce.mult(maxSpeed);
    sForce.sub(vel);
    return sForce;
  }
  
  PVector cohesionForce(ArrayList<Agent> agents) {
    PVector cForce = new PVector(0, 0);
    PVector sum = new PVector(0, 0);
    int num = 0;
    for (Agent agent : agents) {
      if (agent.atGoal) {
        continue;
      }
      float dist = PVector.dist(pos, agent.pos);
      if (dist > 0 && dist < neighborhood) {
        sum.add(agent.pos);
        num += 1;
      }
    }
    if (num > 0) {
      sum.mult(1.0 / num);
      cForce = PVector.sub(sum, pos);
      cForce.normalize();
      cForce.mult(maxSpeed);
      cForce.sub(vel);
    }
    return cForce;
  }
  
  PVector alignForce(ArrayList<Agent> agents) {
    PVector aForce = new PVector(0, 0);
    PVector sum = new PVector(0, 0);
    int num = 0;
    for (Agent agent : agents) {
      if (agent.atGoal) {
        continue;
      }
      float dist = PVector.dist(pos, agent.pos);
      if (dist > 0 && dist < neighborhood) {
        sum.add(agent.vel);
        num += 1;
      }
    }
    if (num > 0) {
      sum.mult(1.0 / num);
      aForce.set(sum);
      aForce.normalize();
      aForce.mult(maxSpeed);
      aForce.sub(vel);
    }
    return aForce;
  }
  
  PVector obstacleForce(ArrayList<Obstacle> obs) {
    PVector oForce = new PVector(0, 0);
    for (Obstacle ob: obs) {
      PVector dif = PVector.sub(pos, ob.pos);
      float mag = dif.mag();
      if (mag > 0 && mag < ob.configRadius + obstacleBuffer) {
        dif.normalize();
        dif.mult(1.0 / mag);
        oForce.add(dif);
      }
    }
    oForce.normalize();
    oForce.mult(maxSpeed);
    oForce.sub(vel);
    return oForce;
  }
  
  boolean isCollidingWith(ArrayList<Obstacle> obs) {
    for (Obstacle ob : obs) { 
      float xDist = ob.pos.x - pos.x;
      float yDist = ob.pos.y - pos.y;
      if ( xDist*xDist + yDist*yDist < ob.configRadius*ob.configRadius) {
        return true;
      }
    }
    return false;
  }
  
  void display() {
    fill(col.r, col.g, col.b);
    noStroke();
    ellipse(pos.x, pos.y, 2*radius, 2*radius);
  }
  
  void displayPath() {
    for (int i = 1; i < path.size(); i++) {
      stroke(col.r, col.g, col.b, 1);
      strokeWeight(3);
      line(path.get(i-1).pos.x, path.get(i-1).pos.y, path.get(i).pos.x, path.get(i).pos.y); 
    }
  }
}


// --------------- OBSTACLE CLASS --------------- 
class Obstacle {
  PVector pos;
  float radius;
  float configRadius;
  Color col;
  
  Obstacle(float x, float y, float r, float cr, Color c) {
    pos = new PVector(x, y);
    radius = r;
    configRadius = cr;
    col = c;
  }
  
  boolean isCollidingWith(ArrayList<Obstacle> obs) {
    for (Obstacle ob : obs) { 
      float xDist = ob.pos.x - pos.x;
      float yDist = ob.pos.y - pos.y;
      if (xDist*xDist + yDist*yDist < ob.configRadius*ob.configRadius && ob != this) {
        return true;
      }
    }
    return false;
  }
  
  boolean isInside(float x, float y) {
    return (x - pos.x)*(x - pos.x) + (y - pos.y)*(y - pos.y) < configRadius*configRadius; 
  }
  
  boolean intersect(PVector a, PVector b) {
    PVector segment = PVector.sub(a, b);
    PVector toObstacle = PVector.sub(b, pos);
    float A = abs(segment.dot(segment));
    float B = 2 * toObstacle.dot(segment);
    float C = abs(toObstacle.dot(toObstacle)) - configRadius*configRadius;
    float disc = B*B - 4*A*C;
    if (disc < 0) {
      return false;
    } 
    disc = sqrt(disc);
    float x1 = (-B - disc) / (2*A);
    float x2 = (-B + disc) / (2*A);
    if (x1 >= 0 && x1 <= 1) {
      return true;
    }
    if (x2 >= 0 && x2 <= 1) {
      return true;
    } 
    return false;
  }
  
  void display() {
    fill(col.r, col.g, col.b);
    noStroke();
    ellipse(pos.x, pos.y, 2*radius, 2*radius);
  }
}


// --------------- COLOR CLASS --------------- 
class Color {
  float r;
  float g;
  float b;
  
  Color(float red, float green, float blue) {
    r = red;
    g = green;
    b = blue;
  }
}


// --------------- NODE CLASS --------------- 
class Node {
  PVector pos;
  ArrayList<Node> neighbors;
  boolean visited = false;
  
  Node(float x, float y) {
    pos = new PVector(x, y);
    neighbors = new ArrayList();
  }
}


// --------------- PATH CLASS --------------- 
class Path {
  ArrayList<Node> path = new ArrayList<Node>();
  float fscore = 0;  
}


// --------------- HELPER FUNCTION --------------- 
void keyPressed() {
  if (key == 'g') {
    PLACE = false;
    PLANNING = true;
  }
}
