Advanced Ratatouille:mouse:
====================
##Basic idea:
1. use flood fill algorithm(see the slides) to search the center<br />
2. when at the center, reassign distance values of all cells based on <br />
   the actual distance from the cell to the center.<br />
3. construct the shortest 'route' between center and origin<br />
4. use the 'route' to run back home to finish search run<br />
5. use the 'route' to run to center for speed run<br />
6. use the 'route' to run back home for speed run<br />

##Useful commands:
compile source code: `$ make` <br />
run it:`$ ./run [-m N] [-p] [-v] [-d]`   <br />
options: <br />
	`-m N`	specify which maze to run with (`N` is the id number of the maze)<br />
	`-p`		pause at every move<br />
	`-v`		verbose. Output useful debugging information<br />
	`-d`		demo. Only perform first run (search run)<br />

`$ make clean` before we wanna compile updated version <br />	
if we wanna run left follower, use `$ make leftfollower` and `$ ./LfRun [-m N] [-p]` <br />

Gihub has emoji.:+1::+1::+1:
