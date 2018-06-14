## Maps101

This program displays various road maps and allows users to find the shortest path between any two nodes. For example, if the user loaded the San Francisco map and selected two nodes, your program should display the best route.<br/>

We are testing 4 alogrithms here and 1 extension:<br/>
1.) BFS - will yiel path with fewest steps, not shortest or least expensive!<br/>
2.) Dijkstra - shorest path <br/>
3.) A* - was build upon Dijkstra, adds a heuristic on the cost estimate when equing in priority queue. Faster then Dijkstra.<br/>
3.b.) Bidirectional A* - this implementation works only for undirected graphs. Will be faster than A*<br/>
4.) Alternative Route<br/>

#### Personalize?
You can add your own maps, by uplaoding a .png file into the res folder. You further need to specifiy the locatio of the nodes and the directed branches among your nodes in a .txt file with it. Look at one of the porvided exmaples for the formatting.

#### What do you need?
The GUI uses Java Console. Make sure everything is up to date.
