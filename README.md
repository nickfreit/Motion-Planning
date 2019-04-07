# Motion-Planning
<p>
The multi_agent_planning.pde file is the main file for the simulation of multiple agents path planning. 
The agents globally plan using a PRM, and locally interact with each other using the Boid model created by Craig Reynolds. 
The simulation will start with 25 agents in the bottom right, a goal in the top left, and 25 red obstacles. 
More obstacles can be added by clicking with the mouse. 
To add more agents, press A to spawn an agent at the current mouse position. 
To add more goals, press F to spawn a goal at the current mouse position. Any subsequent agents added will 
have the same color as this goal and seek it. The goal will not appear until agents are added that will seek it. 
Press G to start the simulation.    </p> <p> 
The rrt_path_planning.pde file shows one agent navigating with an RRT. To add more obstacles, just click. 
Press G to start the simulation. </p> <p>
The path_planning_astar.pde file shows one agent and displays the time it takes for that agent to plan a path 
using both A* search and uniform cost search. This allows you to compare search methods. Again press G to start. </p>
