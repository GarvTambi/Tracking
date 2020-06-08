# Short Distance Tracking

To achieve this goal I used Repast J software to build an agent simulation. I implemented several 
types of delivery drivers to compare different AI algorithms’ performance.

I choose to implement two types of AI algorithms: Breadth-first search and A*.

You can run the simulation to see the simulation.

### What to download to run the simulation

First download and install ''Repast J'' (one of the most used open-source, agent-based modeling 
and simulation toolkit)

Second, download an IDE for JAVA code. I choose Eclipse.

### How to run the simulation?

Download the whole project with libraries under ''logist'' folder.

Then, in Eclipse (do the similar steps for your own IDE if you choose another):

- Right click on the project

- Go to ''Build Path'' / ''Configure Build Path''

- Add External JARs

- Browse to logist/logist.jar

- Apply and close

- Go to ''Run''/''Run configuration''

- Project : deliberative, Main class: logist.LogistPlatform, Arguments: config/deliberative.xml deliberative-astar
(Note: it will run only one agent with A* algorithm, you can run multiple agents by adding twice: ''deliberative-astar'' after the xml)

You will run the delivery simulation with 7 tasks to pick up, you can change it in ''config/deliberative.xml''
