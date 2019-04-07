import java.util.*;


// --------------- GLOBAL VARS --------------- 
int NUM_OBSTACLES = 50;
float OBSTACLE_RADIUS = 20;
float AGENT_RADIUS = 5;
float GOAL_RADIUS = 20;
float AGENT_X = 960.0 / 10;
float AGENT_Y = 9 * 960.0 / 10;
float GOAL_X = 9 * 960.0 / 10;
float GOAL_Y = 960.0 / 10;

boolean PLANNING = false;
boolean PLACE = true;

Color OBSTACLE_COLOR = new Color(255, 0, 0);
Color AGENT_COLOR = new Color(0, 0, 255);
Color GOAL_COLOR = new Color(0, 0, 255);

Agent AGENT;
ArrayList<Obstacle> OBSTACLES = new ArrayList<Obstacle>(NUM_OBSTACLES);
Obstacle GOAL;
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
  AGENT = new Agent(width-AGENT_X + random(-50, 50), AGENT_Y  + random(-50, 50), 0, 0, AGENT_RADIUS, AGENT_COLOR);
  GOAL = new Obstacle(width-GOAL_X, GOAL_Y, GOAL_RADIUS, 0, GOAL_COLOR);
 while(AGENT.isCollidingWith(OBSTACLES)) {
    AGENT = new Agent(width-AGENT_X + random(-50, 50), AGENT_Y  + random(-50, 50), 0, 0, AGENT_RADIUS, AGENT_COLOR);
  }
  
}


// --------------- DRAW --------------- 
void draw() {
  background(200);
  
  if (mousePressed && PLACE) {
    OBSTACLES.add(new Obstacle(mouseX, mouseY, OBSTACLE_RADIUS, OBSTACLE_RADIUS + AGENT_RADIUS, OBSTACLE_COLOR));
  }
  if (PLANNING) {
    generateRRT(GRAPH, OBSTACLES, AGENT, GOAL);
    setPath(GRAPH, AGENT);
    PLANNING = false;
  } else if (!PLANNING && !PLACE) {
    AGENT.checkGoal(GOAL);
    AGENT.update(0.01, OBSTACLES);

  }
  
  AGENT.display();
  AGENT.displayPath();
  
  for (Obstacle ob : OBSTACLES) {
    ob.display();
  }
  
  GOAL.display();
  
  displayRRT(GRAPH);
}

boolean inObstacle(ArrayList<Obstacle> obs, float x, float y) {
  for (Obstacle ob : obs) {
    if (ob.isInside(x, y)) {
      return true;
    }
  }
  return false;
}


// --------------- COLLISION CHECKING --------------- 
boolean intersectsObstacle(ArrayList<Obstacle> obs, PVector p1, PVector p2) {
  for (Obstacle ob: obs) {
    if (ob.intersect(p1, p2)) {
      return true;
    }
  }
  return false;
}


// --------------- RRT --------------- 
void setPath(ArrayList<Node> graph, Agent agent) {
  Node goal = graph.get(graph.size() - 1);
  Node it = goal;
  ArrayList<Node> backwardsPath = new ArrayList<Node>();
  while (it.parent != null) {
    backwardsPath.add(it);
    it = it.parent;
  }
  for (int i = backwardsPath.size() - 1; i >= 0; i--) {
    agent.path.add(backwardsPath.get(i));
  }
}

void generateRRT(ArrayList<Node> graph, ArrayList<Obstacle> obs, Agent agent, Obstacle goal) {
  Node start = new Node(agent.pos.x, agent.pos.y, null); 
  graph.add(start);
  int count = 0;
  while (true) {
    if (count % 50 == 0) {
      for (Node n : graph) {
        if (!intersectsObstacle(obs, n.pos, goal.pos)) {
          graph.add(new Node(goal.pos.x, goal.pos.y, n));
          return;
        }
      }
    }
    
    PVector newPos = new PVector(random(0, width), random(0, height));
    float minDist = width + height;
    int minIndex = graph.size() - 1;
    for (int i = 0; i < graph.size(); i++) {
      float dist = PVector.dist(graph.get(i).pos, newPos);
      if (dist < minDist ) {
        minDist = dist;
        minIndex = i;
      }
    }
    Node nearest = graph.get(minIndex);
    PVector dir = PVector.sub(newPos, nearest.pos);
    while (intersectsObstacle(obs, nearest.pos, newPos)) {
      newPos.x -= 0.05 * dir.x;
      newPos.y -= 0.05 * dir.y;
    }
    
    Node newNode = new Node(newPos.x, newPos.y, nearest);
    graph.add(newNode);
  }
}

void displayRRT(ArrayList<Node> graph) {
  for (Node node : graph) {
    strokeWeight(2);
    stroke(0, 50);
    if (node.parent != null) {
      line(node.pos.x, node.pos.y, node.parent.pos.x, node.parent.pos.y);
    }
  }
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
  float separation = 20;
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
  
  void update(float dt, ArrayList<Obstacle> obs) {
    PVector pForce = pathForce(obs);
  
    acc.add(pForce.mult(7));
    
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
      stroke(col.r, col.g, col.b);
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
  Node parent;
  boolean visited = false;
  
  Node(float x, float y, Node parent) {
    pos = new PVector(x, y);
    neighbors = new ArrayList();
    this.parent = parent;
  }
}


// --------------- HELPERS --------------- 
void keyPressed() {
  if (key == 'g') {
    PLACE = false;
    PLANNING = true;
  }
}
