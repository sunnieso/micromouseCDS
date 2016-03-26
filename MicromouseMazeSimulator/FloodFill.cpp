#include <iostream>
#include <cstdlib>  // atoi
#include "Maze.h"
#include "MazeDefinitions.h"
#include "PathFinder.h"
#include <stack>
#include <queue>
#define INFINITY 10*MazeDefinitions::MAZE_LEN

/**
 * Our implementation.
 * Use floodfill algorithm to traverse the maze and to find shortest path.
 * For clarity, sost class functions implementation is at the bottom of this file.
 * 
 * Basic idea:
 * [1] use flood fill algorithm(see the slides) to search the center
 * [2] when at the center, reassign distance values of all cells based on 
 *     the actual distance from the cell to the center.
 * [3] construct the shortest 'route' between center and origin
 * [4] use the 'route' to run back home to finish search run
 * [5] use the 'route' to run to center for speed run
 * [6] use the 'route' to run back home for speed run 
 **  

   Initial values:
   Manhattan Distance = 
   use getMDistance to obtain this.
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 15 |14 |13 |12 |11 |10 | 9 | 8 | 7 | 7 | 8 | 9 |10 |11 |12 |13 |14 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 14 |13 |12 |11 |10 | 9 | 8 | 7 | 6 | 6 | 7 | 8 | 9 |10 |11 |12 |13 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 13 |12 |11 |10 | 9 | 8 | 7 | 6 | 5 | 5 | 6 | 7 | 8 | 9 |10 |11 |12 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 12 |11 |10 | 9 | 8 | 7 | 6 | 5 | 4 | 4 | 5 | 6 | 7 | 8 | 9 |10 |11 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 11 |10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |10 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 9  | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 8  | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |  
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 7  | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 6  | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 5  | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 4  |10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |10 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 3  |11 |10 | 9 | 8 | 7 | 6 | 5 | 4 | 4 | 5 | 6 | 7 | 8 | 9 |10 |11 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 2  |12 |11 |10 | 9 | 8 | 7 | 6 | 5 | 5 | 6 | 7 | 8 | 9 |10 |11 |12 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 1  |13 |12 |11 |10 | 9 | 8 | 7 | 6 | 6 | 7 | 8 | 9 |10 |11 |12 |13 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 0  |14 |13 |12 |11 |10 | 9 | 8 | 7 | 7 | 8 | 9 |10 |11 |12 |13 |14 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
     0    1   2   3   4   5   6   7   8   9  10  11  12  13  14  15 
 */

// similar to LeftWallFollower. JK, not really.
class FloodFill : public PathFinder {
public:


    /*******
     * 
     * Helper Structures
     *
     **/

    // algorithm mode
    enum Mode
    {
        MODE_SEARCH,        // finding distances. Mouse should be at the center when done searching.
        MODE_BACK_HOME,     // After we reach center, find a way to go back home.    
        MODE_FAST,          // second run to center. Speed run.
        MODE_FAST_BACK_HOME // second run to origin
    };

    /**
     * Cell structure contains information of a single cell of the map.
     * info includes:  
     * bool northWall, southWall, eastWall, westWall
     *      4 adjacent wall status.
     *      When the mouse walks through the cell, it records the adjacent wall status.
     * unsigned distance:
     *      initially, 'distance' hold Manhatan distance(see the above illustration). 
     *      This value will be modified after applying floodfill algorithm (step 2).
     * bool visited:
     *      Tells us whether or not the mouse has visited this cell. We need this because 
     *      only the cell that the mouse has visited has valid wall status. So before we check
     *      wall status, we need to make sure that the cell is visited.
     */
    struct Cell{
        Cell(){
            northWall = true;
            southWall = true;
            eastWall = true;
            westWall = true;
            visited = false;
        }
        void setWall(Dir d, bool set = false){
            switch(d){
                case NORTH: 
                    northWall = set;
                    break;
                case SOUTH: 
                    southWall = set;
                    break;
                case EAST: 
                    eastWall = set;
                    break;
                case WEST: 
                    westWall = set;
                    break;
            }
        }
        void setDistance(unsigned newDistance){
            distance = newDistance;
        }

        bool northWall;
        bool southWall;
        bool eastWall;
        bool westWall;
        bool visited;
        unsigned distance;
        unsigned cx, cy;
    };

    // initial setup
    FloodFill(bool shouldPause = false, bool shouldPrint = false, bool shouldDemo = false) : pause(shouldPause), verbose(shouldPrint), demo(shouldDemo) {
        // define initial heading to be north
        currHeading = NORTH;
        // default mode is search mode.
        mode = MODE_SEARCH;
        // construct map with Manhatan distances
        for (int r = 0; r != MazeDefinitions::MAZE_LEN; r++){
            for (int c = 0; c != MazeDefinitions::MAZE_LEN; c++){
                map[r][c].distance=getManDistance(r,c);
                map[r][c].cx = r;
                map[r][c].cy = c;
            }
        }      
    }


    // boss function 
    MouseMovement nextMovement(unsigned x, unsigned y, const Maze &maze) {
        // get current cell wall status. (using IR sensors)
        frontWall = maze.wallInFront();
        leftWall  = maze.wallOnLeft();
        rightWall = maze.wallOnRight();
        // obtain the distance of the current cell that the mouse is at.
        currMDistance = map[x][y].distance;

        // obtain the current heading
        setHead(currHeading, retval);
        // currHeading = maze.getHeading();   // North | South | East | West

        const unsigned mid = MazeDefinitions::MAZE_LEN / 2;

        // Pause at each cell if the user requests it.
        // It allows for better viewing on command line.
        if(pause) {
            std::cout << "Hit enter to continue..., (" << x << "," << y << "), M=" << currMDistance << " head " << currHeading << std::endl;
            std::cin.ignore(10000, '\n');
            std::cin.clear();
        }

        std::cout << maze.draw(5) << std::endl << std::endl;

        // If we somehow miraculously hit the center
        // of the maze, then:
        // [1] if it is just for the demo, then we are done.
        // [2] if it is the search run, then it means we finished searching and we should start heading back home.
        // [3] if it is the speed run, then we are done done !!!! hoo-ray
        if(isAtCenter(x, y)) {
            if(demo){
                std::cout << "Found center! Good enough for the demo, won't try to get back." << std::endl;
                return Finish;
            }
            if(mode == MODE_FAST){
                std::cout << "Fast run half way through!" << std::endl;
                mode = MODE_FAST_BACK_HOME;
                return TurnAround;
            }
            if(mode == MODE_SEARCH){
                clearVisits();
                assign_new_dis(&map[x][y]);
                constructRoute();
                mode = MODE_BACK_HOME;
                return TurnAround;
            }
        }

        // If we hit the start of the maze a second time, then either
        // [1] we couldn't find the center and never will...
        // [2] we've reached end goal of home run.
        if(x == 0 && y == 0) {
            if(mode == MODE_BACK_HOME){
                std::cout << "Back home run finished!" << std::endl;
                mode = MODE_FAST;
                return TurnAround;
            }else if(mode == MODE_SEARCH && visitedStart) {
                std::cout << "Unable to find center, giving up." << std::endl;
                return Finish;
            }else if(mode == MODE_FAST_BACK_HOME){
                std::cout << "Fast run FINISH!!!!\n";
                return Finish;
            } else {
                visitedStart = true;
            }
        }

        // switch to algorithm
        switch(mode){
            case MODE_SEARCH:
                SearchMode(x,y);
            break;
            case MODE_BACK_HOME:
            case MODE_FAST_BACK_HOME:
                HomeBoundMode(x,y);
            break;
            case MODE_FAST:
                FastMode();
            break;
        }
        return retval;
            
    }

protected:

    // debugging purpose. When specify -v option, output more stuffs.
    bool verbose; 
    // demo. When specify -d option, only run search mode. By default this is false;
    bool demo;

    Mode mode;
    // Helps us determine if we've made a loop around the maze without finding the center.
    bool visitedStart;

    // Indicates we should pause before moving to next cell.
    // Useful for command line usage.
    const bool pause;

    // the current Manhattan distance.
    unsigned currMDistance;
    // current heading of the mouse. 
    Dir currHeading;

    unsigned minMDistance;
    MouseMovement retval;

    // keep track of current neighboring walls status
    bool frontWall;
    bool leftWall;
    bool rightWall;

    // map that holds distances. initially it contains manhatan distances, later on will be modified using floodfill algorithm.
    Cell map[MazeDefinitions::MAZE_LEN][MazeDefinitions::MAZE_LEN];

    // construct fastest route (stacks) for homebound mode and fast mode
    // preprocess: construct routeSt1, starting from origin and ending at center
    // homebound: pop instructions from routeSt1 and push them in routeSt2
    // fast run: pop instructions from routeSt2 and push them in routeSt1. 
    std::stack<MouseMovement> routeSt1;
    std::stack<MouseMovement> routeSt2;


    /*******
     * 
     * Member Functions Declaration
     * (Implementation is at the bottom of the file)
     *
     **/

    // In the case that we can't have access to the heading in Maze.h, this function helps us keep track of the current heading.
    // We call this function at the beginning of nextmovement() so we have updated heading. 
    void setHead(Dir &oldHeading, MouseMovement insn);

    // same as left follower. Check if the mouse is at the center of the maze.
    bool isAtCenter(unsigned x, unsigned y) const;

    // only used for initial map construction
    // calculate the Manhattan distances of each cell. (result == the illustration above)
    unsigned getManDistance(unsigned x, unsigned y);

    // reset visit history of all cells.
    void clearVisits();

    // for search mode step one. Does two things:
    // [1] use front, right, left wall status to find min distance.
    // [2] assign return value.(mouse movement)
    void find_minDistance_and_nextInsn(unsigned x, unsigned y);

    // Used in constructing route. Very similar to 'find_minDistance_and_nextInsn'.
    // use front, right, left wall status to find min distance.
    // The difference is that it only checks the adjacent cells that the mouse has already visited
    void find_minDistance_and_nextInsn_II(unsigned x, unsigned y, Dir funcHeading);

    // use north,south,east,west wall status to find min distance 
    // when isConstructingRoute is set, check only the cells that the mouse has visited.
    Cell* findMinDistance(unsigned cx, unsigned cy, bool isConstructingRoute);

    // Call this after the mouse searched the center for the first time.
    // This function reassign the distance of all cells based on its 'physical' shortest path from the center. (i.e. consider walls)
    // need to call 'clearVisits' before using this function.
    void assign_new_dis(Cell* currCell);

    // After the mouse reached the center for the first time and the distances map has been reassigned,
    // we call this function to construct the 'shortest' route from origin(home) to center.
    // we have two stacks, routeSt1 and routeSt2. routeSt1 stores the instructions needed to traverse from center to home
    // routeSt2 will store the same but in reverse order when we run HomeBoundMode because routeSt2 push whatever routeSt1 pop. 
    void constructRoute();

    // First run searching center
    void SearchMode(unsigned x, unsigned y);

    // First run going back home and speed run running back home
    void HomeBoundMode(unsigned x, unsigned y);

    // Speed run running to center
    void FastMode();

};


int main(int argc, char * argv[]) {
    MazeDefinitions::MazeEncodingName mazeName = MazeDefinitions::MAZE_CAMM_2012;
    bool pause = false;
    bool verbose = false;
    bool demo = false;
    // Since Windows does not support getopt directly, we will
    // have to parse the command line arguments ourselves.

    // Skip the program name, start with argument index 1
    for(int i = 1; i < argc; i++) {
        if(strcmp(argv[i], "-m") == 0 && i+1 < argc) {
            int mazeOption = atoi(argv[++i]);
            if(mazeOption < MazeDefinitions::MAZE_NAME_MAX && mazeOption > 0) {
                    mazeName = (MazeDefinitions::MazeEncodingName)mazeOption;
            }
        } else if(strcmp(argv[i], "-p") == 0) {
            pause = true;
        } else if(strcmp(argv[i], "-v") == 0) {
            verbose = true;
        } else if(strcmp(argv[i], "-d") == 0) {
            demo = true;
        } else {
            std::cout << "Usage: " << argv[0] << " [-m N] [-p] [-v] [-d]" << std::endl;
            std::cout << "\t-m N will load the maze corresponding to N, or 0 if invalid N or missing option" << std::endl;
            std::cout << "\t-p will wait for a newline in between cell traversals" << std::endl;
            std::cout << "\t-v will output useful debugging info" << std::endl;
            std::cout << "\t-d will only perform search run" << std::endl;
            return -1;
        }
    }

    FloodFill floodfill(pause, verbose, demo);
    Maze maze(mazeName, &floodfill);
    std::cout << maze.draw(5) << std::endl << std::endl;

    maze.start();
}


/**
 *
 * FloodFill member functions implementation
 *
 */

void FloodFill::setHead(Dir &oldHeading, MouseMovement insn){
    switch(insn){
        case TurnAround:
            oldHeading = opposite(oldHeading);
            return;
        case TurnClockwise:
            oldHeading = clockwise(oldHeading);
            return;
        case TurnCounterClockwise:
            oldHeading = counterClockwise(oldHeading);
            return;
    }
}

 bool FloodFill::isAtCenter(unsigned x, unsigned y) const {
    unsigned midpoint = MazeDefinitions::MAZE_LEN / 2;

    if(MazeDefinitions::MAZE_LEN % 2 != 0) {
        return x == midpoint && y == midpoint;
    }

    return  (x == midpoint     && y == midpoint    ) ||
    (x == midpoint - 1 && y == midpoint    ) ||
    (x == midpoint     && y == midpoint - 1) ||
    (x == midpoint - 1 && y == midpoint - 1);
}


// only used for initial map construction
unsigned FloodFill::getManDistance(unsigned x, unsigned y){
    // error checking
    if (x >= MazeDefinitions::MAZE_LEN || y >= MazeDefinitions::MAZE_LEN){
        if(verbose)
            std::cout << "Error: invalid (x,y) coordinates." << std::endl;
        return -1;
    }
    unsigned mid = MazeDefinitions::MAZE_LEN / 2 ;
    // lower left section
    if (x < mid && y < mid)
        return mid + mid - x - y - 2;
    // upper left section
    if (x < mid)
        return y - x - 1;
    // lower right section
    if (y < mid)
        return x - y - 1;
    // upper right section
    return x + y - mid - mid;
}
// reset visit history of all cells.
void FloodFill::clearVisits(){
    for (int r = 0; r != MazeDefinitions::MAZE_LEN; r++){
        for (int c = 0; c != MazeDefinitions::MAZE_LEN; c++){
            map[r][c].visited = false;
        }
    }
}

// Call this after the mouse searched the center for the first time.
// This function reassign the distance of all cells based on its 'physical' shortest path from the center. (i.e. consider walls)
// need to call 'clearVisits' before using this function.
void FloodFill::assign_new_dis(Cell* currCell){
    currCell->distance = 0;
    std::queue<Cell*> qu;
    qu.push(currCell);
    while(qu.front() != &map[0][0]){
        currCell = qu.front();
        currCell->visited = true;
        unsigned newDis = currCell->distance +1;
        if(verbose){
            std::cout << "qu.front() = (" << qu.front()->cx << "," << qu.front()->cy << "). newDis = " << currCell->distance << "\n"; 
        }
        qu.pop();
        if(!currCell->northWall && !map[currCell->cx][currCell->cy+1].visited){
            map[currCell->cx][currCell->cy+1].distance = newDis;
            qu.push(&map[currCell->cx][currCell->cy+1]);
        }

        if(!currCell->southWall && !map[currCell->cx][currCell->cy-1].visited){
            map[currCell->cx][currCell->cy-1].distance = newDis;
            qu.push(&map[currCell->cx][currCell->cy-1]);
        }

        if(!currCell->eastWall && !map[currCell->cx+1][currCell->cy].visited){
            map[currCell->cx+1][currCell->cy].distance = newDis;
            qu.push(&map[currCell->cx+1][currCell->cy]);
        }

        if(!currCell->westWall && !map[currCell->cx-1][currCell->cy].visited){
            map[currCell->cx-1][currCell->cy].distance = newDis;
            qu.push(&map[currCell->cx-1][currCell->cy]);
        }
        
    }
    qu.front()->visited = true;
    if(verbose){
        std::cout << "qu.front() = (" << qu.front()->cx << "," << qu.front()->cy << "). newDis = " << qu.front()->distance << "\n"; 
        std::cout << "Done assigning new distances.\n";
    }
}


// for search mode step one. Does two things:
// [1] use front, right, left wall status to find min distance.
// [2] assign return value.(mouse movement)
void FloodFill::find_minDistance_and_nextInsn(unsigned x, unsigned y){

    // calculate the x y coordinates of the 'front' cell.
    unsigned forwardX = 0;
    unsigned forwardY = 0;
    switch(currHeading){
        case NORTH:
            forwardY = 1;
            break;
        case SOUTH:
            forwardY = -1;
            break;
        case EAST:
            forwardX = 1;
            break;
        case WEST:
            forwardX = -1;
            break;
    }
   
    minMDistance = currMDistance;

    // In the below three if clauses, we check the distance of the adjacent cell and 
    // set the wall status of the adjacent cell & current cell.

    // check the Mdistance of the grid on the left
    if(!leftWall){
        map[x][y].setWall(counterClockwise(currHeading));
        map[x - forwardY][y + forwardX].setWall(clockwise(currHeading));
        if( map[x - forwardY][y + forwardX].distance <= minMDistance){
            minMDistance = map[x - forwardY][y + forwardX].distance;
            retval = TurnCounterClockwise;
        }
    }

    // check the Mdistance of the grid on the right
    if(!rightWall){
        map[x][y].setWall(clockwise(currHeading));
        map[x + forwardY][y - forwardX].setWall(counterClockwise(currHeading));
        if(map[x + forwardY][y - forwardX].distance <= minMDistance){
            minMDistance = map[x + forwardY][y - forwardX].distance;
            retval = TurnClockwise;
        }
    }

    // check the Mdistance of the grid at the front
    if(!frontWall){
        // set wall status of the current cell
        map[x][y].setWall(currHeading);
        // set wall status of the cell at the front
        map[x+ forwardX][y+ forwardY].setWall(opposite(currHeading));

        // find min distance
        if(map[x + forwardX][y + forwardY].distance <= minMDistance){
            minMDistance = map[x + forwardX][y + forwardY].distance;
            retval = MoveForward;
        }
    }
}


// Used in constructing route. Very similar to 'find_minDistance_and_nextInsn'.
// use front, right, left wall status to find min distance.
// The difference is that it only checks the adjacent cells that the mouse has already visited
void FloodFill::find_minDistance_and_nextInsn_II(unsigned x, unsigned y, Dir funcHeading){

    // calculate the x y coordinates of the 'front' cell.
    unsigned forwardX = 0;
    unsigned forwardY = 0;
    bool func_frontWall, func_leftWall, func_rightWall;
    switch(funcHeading){
        case NORTH:
            func_frontWall = map[x][y].northWall;
            func_rightWall = map[x][y].eastWall;
            func_leftWall = map[x][y].westWall;
            forwardY = 1;
            break;
        case SOUTH:
            func_frontWall = map[x][y].southWall;
            func_rightWall = map[x][y].westWall;
            func_leftWall = map[x][y].eastWall;
            forwardY = -1;
            break;
        case EAST:
            func_frontWall = map[x][y].eastWall;
            func_rightWall = map[x][y].southWall;
            func_leftWall = map[x][y].northWall;
            forwardX = 1;
            break;
        case WEST:
            func_frontWall = map[x][y].westWall;
            func_rightWall = map[x][y].northWall;
            func_leftWall = map[x][y].southWall;
            forwardX = -1;
            break;
    }
    
    minMDistance = map[x][y].distance;

    // In the below three if clauses, we check the distance of the adjacent cell which the mouse has already visited.

    if(verbose){
        std::cout << "\nII: (" << x << "," << y << "):\n";
        std::cout << "II: current distance=" <<  map[x][y].distance << "\n";
        std::cout << "II: func_frontWall=" << func_frontWall << std::endl;
        std::cout << "II: func_rightWall=" << func_rightWall << std::endl;
        std::cout << "II: func_leftWall=" << func_leftWall << std::endl;
    }
    // check the Mdistance of the grid on the left
    if(!func_leftWall && map[x - forwardY][y + forwardX].visited){
        if(verbose){
            std::cout << "II: (" << x - forwardY << "," << y + forwardX << "):visited\n";
            std::cout << "II: (" << x - forwardY << "," << y + forwardX << ").distance=" <<  map[x - forwardY][y + forwardX].distance << "\n";
        }
        if( map[x - forwardY][y + forwardX].distance <= minMDistance){
            minMDistance = map[x - forwardY][y + forwardX].distance;
            retval = TurnCounterClockwise;
        }
    }

    // check the Mdistance of the grid on the right
    if(!func_rightWall && map[x + forwardY][y - forwardX].visited){
        if(verbose){
            std::cout << "II: (" << x + forwardY << "," << y - forwardX << "):visited\n";
            std::cout << "II: (" << x + forwardY << "," << y - forwardX << ").distance=" <<  map[x + forwardY][y - forwardX].distance << "\n";
        }
        if(map[x + forwardY][y - forwardX].distance <= minMDistance){
            minMDistance = map[x + forwardY][y - forwardX].distance;
            retval = TurnClockwise;
        }
    }

    // check the Mdistance of the grid at the front
    if(!func_frontWall && map[x + forwardX][y + forwardY].visited){
        if(verbose){
            std::cout << "II: (" << x + forwardX << "," << y + forwardY << "):visited\n";
            std::cout << "II: (" << x + forwardX << "," << y + forwardY << ").distance=" <<  map[x + forwardX][y + forwardY].distance << "\n";
        }
        // find min distance
        if(map[x + forwardX][y + forwardY].distance <= minMDistance){
            minMDistance = map[x + forwardX][y + forwardY].distance;
            retval = MoveForward;
        }
    }

    if(verbose){
            std::cout << "II: obtained minMDistance" <<  minMDistance << "\n";
    }
}


// use north,south,east,west wall status to find min distance 
// when isConstructingRoute is set, check only the cells that the mouse has visited.
FloodFill::Cell* FloodFill::findMinDistance(unsigned cx, unsigned cy, bool isConstructingRoute = false){
    minMDistance = INFINITY;
    Cell* retCell = NULL;

    if(!map[cx][cy].northWall && (map[cx][cy+1].distance <= minMDistance)){
        if(isConstructingRoute){
            if(map[cx][cy+1].visited){
                if(verbose){
                    std::cout << "findMin: (" << cx  << "," << cy+1 << "):visited\n";
                    std::cout << "findMin: (" << cx << "," << cy+1 << ").distance=" <<  map[cx][cy+1].distance << "\n";
                }
                minMDistance = map[cx][cy+1].distance;
                retCell = &map[cx][cy+1];
            }
        }else{
            minMDistance = map[cx][cy+1].distance;
        }
    }
    // check the distance of the grid in the south
    if(!map[cx][cy].southWall && (map[cx][cy-1].distance <= minMDistance)){
        if(isConstructingRoute){
            if(map[cx][cy-1].visited){
                if(verbose){
                    std::cout << "findMin: (" << cx  << "," << cy-1 << "):visited\n";
                    std::cout << "findMin: (" << cx << "," << cy-1 << ").distance=" <<  map[cx][cy-1].distance << "\n";
                }
                minMDistance = map[cx][cy-1].distance;
                retCell = &map[cx][cy-1];
            }
        }else{
            minMDistance = map[cx][cy-1].distance;
        }
    }
    // check the distance of the grid in the east
    if(!map[cx][cy].eastWall && (map[cx+1][cy].distance <= minMDistance)){
        if(isConstructingRoute){
            if (map[cx+1][cy].visited){
                if(verbose){
                    std::cout << "findMin: (" << cx+1  << "," << cy << "):visited\n";
                    std::cout << "findMin: (" << cx+1 << "," << cy << ").distance=" <<  map[cx+1][cy].distance << "\n";
                }
                minMDistance = map[cx+1][cy].distance;
                retCell = &map[cx+1][cy];
            }
        }else{
            minMDistance = map[cx+1][cy].distance;
        }
    }
    // check the distance of the grid in the west
    if(!map[cx][cy].westWall && (map[cx-1][cy].distance < minMDistance)){
        if(isConstructingRoute){
            if( map[cx-1][cy].visited){
                if(verbose){
                    std::cout << "findMin: (" << cx-1  << "," << cy << "):visited\n";
                    std::cout << "findMin: (" << cx-1 << "," << cy << ").distance=" <<  map[cx-1][cy].distance << "\n";
                }
                minMDistance = map[cx-1][cy].distance;
                retCell = &map[cx-1][cy];
            }
        }else{
            minMDistance = map[cx-1][cy].distance;
        }
    }
    if(verbose){
        std::cout << "final minDistance=" <<  minMDistance << "\n";
        if(retCell != NULL)
            std::cout << "final retCell= (" <<  retCell->cx << "," << retCell->cy << ")" << "\n";
    }
    return retCell;

}

// After the mouse reached the center for the first time and the distances map has been reassigned,
// we call this function to construct the 'shortest' route from origin(home) to center.
// we have two stacks, routeSt1 and routeSt2. routeSt1 stores the instructions needed to traverse from center to home
// routeSt2 will store the same but in reverse order when we run HomeBoundMode because routeSt2 push whatever routeSt1 pop. 
void FloodFill::constructRoute(){
    unsigned forwardX = 0;
    unsigned forwardY = 0;
    // error checking
    if(!routeSt1.empty())
        return;

    Cell* currCell = &map[0][0];
    Cell* nextCell;
    Dir funcHeading = NORTH;
    while(currCell->distance != 0){
        find_minDistance_and_nextInsn_II(currCell->cx, currCell->cy, funcHeading);
        routeSt1.push(retval);

        // update Cell and funcHeading 
        if (retval == MoveForward){
            forwardX = 0;
            forwardY = 0;
             // calculate the x y coordinates of the 'front' cell.
            switch(funcHeading){
                case NORTH:
                    forwardY = 1;
                    break;
                case SOUTH:
                    forwardY = -1;
                    break;
                case EAST:
                    forwardX = 1;
                    break;
                case WEST:
                    forwardX = -1;
                    break;
            }
            currCell = &map[currCell->cx+forwardX][currCell->cy+forwardY];
        } else {
            // take care of Turnaround, TurnCounterClockwise and TurnClockwise.
            setHead(funcHeading, retval);
        }
    }
}

// First run searching center
void FloodFill::SearchMode(unsigned x, unsigned y){

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // step 1: Follow Manhattan Distances downwards towards center, noting down any walls as they pass
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    map[x][y].visited = true;
    find_minDistance_and_nextInsn(x,y);
    // if mimMDistance is not changed, then currMDistance is the smallest Mdistance in its neighborhood.
    if(minMDistance != currMDistance)
        return;

    if(verbose)
        std::cout << "Step 1 done...";

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // step 2: Apply floodfill algorithm
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // new stack //Stack of points to be processed (can also use queue)
    std::stack<Cell*> st;

    // push current cell onto stack
    st.push(&map[x][y]);
    Cell* curr; 


    while (!st.empty()){
        curr = st.top();
        st.pop();
        // set current cell to 'being processed'
        curr->visited = true;
        // get current cell coordinates.
        unsigned cx = curr->cx;
        unsigned cy = curr->cy;

        // for debugging purpose
        if(verbose){
            std::cout << "\ncurr=[" << cx << "][" << cy << "],old dis=" <<curr->distance << " ";
        }

        // don’t want to process the end goal
        if (curr->distance == 0)
            continue; 


        // for each neighboring cell of curr:
  
        // Similar to step one, find the smallest distance arourd the neighborhood.
        // assign new value to minMDistance.
        findMinDistance(cx,cy);
        
        
        if(minMDistance == INFINITY) // shouldn't go in here, if for some reason minMDistance is not changed, then just ignore it.
            continue;

        if(minMDistance +1 == curr->distance) // nothing was updated, move on
            continue;
        
        curr->setDistance(minMDistance + 1); // set new minimum distance
         
        
        if(verbose)
            std::cout << "{ calcMin=" << minMDistance << ", new dis= " << curr->distance << ", push ";

        // push every visited, connected neighbor onto stack (neighbors the mouse passed by and has no adjacent wall.)
        if((!map[cx][cy].northWall) && map[cx][cy+1].visited){
            st.push(&map[cx][cy+1]);
            if(verbose)
                std::cout << "[" << cx << "][" << cy+1<< "], ";
        }
        if((!map[cx][cy].southWall) && map[cx][cy-1].visited){
            st.push(&map[cx][cy-1]);
            if(verbose)
                std::cout << "[" << cx << "][" << cy-1 << "], ";
        }
        if((!map[cx][cy].eastWall)  && map[cx+1][cy].visited){
            st.push(&map[cx+1][cy]);
            if(verbose)
                std::cout << "[" << cx+1 << "][" << cy << "], ";
        }
        if((!map[cx][cy].westWall) && map[cx-1][cy].visited){
            st.push(&map[cx-1][cy]);
            if(verbose)
                std::cout << "[" << cx-1 << "][" << cy << "], ";
        }
        if(verbose)
            std::cout << "}";
    }
    // IR sensors can't sense the back wall, so let's turn around.
    retval = TurnAround;
    return;
}

// First run going back home and speed run running back home
void FloodFill::HomeBoundMode(unsigned x, unsigned y){
    if(routeSt1.empty()){
        retval = Wait;
        return;
    }
    retval = routeSt1.top();
    routeSt2.push(retval);
    routeSt1.pop();
    if(retval == TurnClockwise)
        retval = TurnCounterClockwise;
    else if(retval == TurnCounterClockwise)
        retval = TurnClockwise;

}
// Speed run running to center
void FloodFill::FastMode(){
    if(routeSt2.empty()){
        retval = Wait;
        return;
    }
    retval = routeSt2.top();
    routeSt1.push(retval);
    routeSt2.pop();
}
