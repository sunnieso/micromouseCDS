#include <iostream>
#include <cstdlib>  // atoi

#include "Maze.h"
#include "MazeDefinitions.h"
#include "PathFinder.h"
#include <stack>

/**
 * Our implementation.
 * Use floodfill algorithm
 
   Manhattan Distance = 
   Initially, 
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
 11 |10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | ...
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 9  | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1  ...
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 8  | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |  
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 7  | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 6  | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1  ...
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 5  | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2  ...
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 4  |10 | 9 | 8 | 7 | 6 | 5 | 4 | 3  ...
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 3  |11 |10 | 9 | 8 | 7 | 6 | 5 | 4  ...
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 2  |12 |11 |10 | 9 | 8 | 7 | 6 | 5  ...
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 1  |13 |12 |11 |10 | 9 | 8 | 7 | 6 | 6 | 7 | 8 | 9 |10 |11 |12 |13 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
 0  |14 |13 |12 |11 |10 | 9 | 8 | 7 | 7 | 8 | 9 |10 |11 |12 |13 |14 |
    +---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+
     0    1   2   3   4   5   6   7   8   9  10  11  12  13  14  15 
 */

// similar to LeftWallFollower
class FloodFill : public PathFinder {
public:

    // algorithm mode
    enum Mode
    {
        MODE_SEARCH,    // finding distances. Mouse should be at the center when done searching.
        MODE_BACK_HOME, // After we reach center, find a way to go back home.    
        MODE_FAST       // second run. Speed run.
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
            distance = 16;
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
        void setXY(unsigned x, unsigned y){
            cx = x;
            cy = y;
        }

        bool northWall;
        bool southWall;
        bool eastWall;
        bool westWall;
        bool visited;
        unsigned distance;
        unsigned cx, cy;
    };

    FloodFill(bool shouldPause = false, bool shouldPrint = false, bool shouldDemo = false) : pause(shouldPause), verbose(shouldPrint), demo(shouldDemo) {
        // default mode is search mode.
        mode = MODE_SEARCH;
        // construct map
        for (int r = 0; r != MazeDefinitions::MAZE_LEN; r++){
            for (int c = 0; c != MazeDefinitions::MAZE_LEN; c++){
                map[r][c].distance=getManDistance(r,c);
                map[r][c].setXY(r,c);
            }
        }
    }

    // void clearVisited(){
    //     for (int r = 0; r != MazeDefinitions::MAZE_LEN; r++){
    //         for (int c = 0; c != MazeDefinitions::MAZE_LEN; c++){
    //             map[r][c].visited = false;
    //         }
    //     } 
    // }


    // for step one. 
    void find_minDistance_and_nextInsn(unsigned x, unsigned y){

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

        // check the Mdistance of the grid at the front
        if(!frontWall){
            // set wall status of the current cell
            map[x][y].setWall(currHeading);
            // set wall status of the cell at the front
            map[x+ forwardX][y+ forwardY].setWall(opposite(currHeading));

            // find min distance
            if(map[x + forwardX][y + forwardY].distance < minMDistance){
                minMDistance = map[x + forwardX][y + forwardY].distance;
                retval = MoveForward;
            }
        }
        // check the Mdistance of the grid on the right
        if(!rightWall){
            map[x][y].setWall(clockwise(currHeading));
            map[x + forwardY][y - forwardX].setWall(counterClockwise(currHeading));
            if(map[x + forwardY][y - forwardX].distance < minMDistance){
                minMDistance = map[x + forwardY][y - forwardX].distance;
                retval = TurnClockwise;
            }
        }
        // check the Mdistance of the grid on the left
        if(!leftWall){
            map[x][y].setWall(counterClockwise(currHeading));
            map[x - forwardY][y + forwardX].setWall(clockwise(currHeading));
            if( map[x - forwardY][y + forwardX].distance < minMDistance){
                minMDistance = map[x - forwardY][y + forwardX].distance;
                retval = TurnCounterClockwise;
            }
        }
    }


    // only used for initial map construction
    unsigned getManDistance(unsigned x, unsigned y){
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
    void SearchMode(unsigned x, unsigned y){

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

        // define infinity
        const unsigned INFINITY = 10*MazeDefinitions::MAZE_LEN;

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

            minMDistance = INFINITY;

            // for each neighboring cell of curr:
      
            // Similar to step one, find the smallest distance arourd the neighborhood.

            // check the distance of the grid in the north
            if(!map[cx][cy].northWall && (map[cx][cy+1].distance <= minMDistance)){
                minMDistance = map[cx][cy+1].distance;
            }
            // check the distance of the grid in the south
            if(!map[cx][cy].southWall && (map[cx][cy-1].distance <= minMDistance)){
                minMDistance = map[cx][cy-1].distance;
            }
            // check the distance of the grid in the east
            if(!map[cx][cy].eastWall && (map[cx+1][cy].distance <= minMDistance)){
                minMDistance = map[cx+1][cy].distance;
            }
            // check the distance of the grid in the west
            if(!map[cx][cy].westWall && (map[cx-1][cy].distance < minMDistance)){
                minMDistance = map[cx-1][cy].distance;
            }

            
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

    void HomeBoundMode(unsigned x, unsigned y){

    }

    void FastMode(unsigned x, unsigned y){

    }
    void contructRoute(){
        // push origin
        routeSt1.push(&map[0][0]);
        Cell* curr = routeSt1.top();
        while(routeSt1.top()->distance != 0){
            
        }
    }

    MouseMovement nextMovement(unsigned x, unsigned y, const Maze &maze) {
        // get current cell wall status. (using IR sensors)
        frontWall = maze.wallInFront();
        leftWall  = maze.wallOnLeft();
        rightWall = maze.wallOnRight();

        currMDistance = map[x][y].distance;
        currHeading = maze.getHeading();   // North | South | East | West

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
        // of the maze, just terminate and celebrate!
        if(isAtCenter(x, y)) {
            if(demo){
                std::cout << "Found center! Good enough for the demo, won't try to get back." << std::endl;
                return Finish;
            }
            if(mode == MODE_FAST){
                std::cout << "Fast run finished!" << std::endl;
                return Finish;
            }
            if(mode != MODE_BACK_HOME){
                mode = MODE_BACK_HOME;
            }
        }

        // If we hit the start of the maze a second time, then either
        // [1] we couldn't find the center and never will...
        // [2] we've reached end goal of home run.
        if(x == 0 && y == 0) {
            if(mode == MODE_BACK_HOME){
                std::cout << "Back home run finished!" << std::endl;
                mode = MODE_FAST;
            }else if(visitedStart) {
                std::cout << "Unable to find center, giving up." << std::endl;
                return Finish;
            } else {
                visitedStart = true;
            }
        }


        switch(mode){
            case MODE_SEARCH:
                SearchMode(x,y);
            break;
            case MODE_BACK_HOME:
                HomeBoundMode(x,y);
            break;
            case MODE_FAST:
                FastMode(x,y);
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
    // homebound: pop cells from routeSt1 and push them in routeSt2
    // fast run: pop cell from routeSt2 and push them in routeSt1. 
    stack<Cell*> routeSt1;
    stack<Cell*> routeSt2;

    bool isAtCenter(unsigned x, unsigned y) const {
        unsigned midpoint = MazeDefinitions::MAZE_LEN / 2;

        if(MazeDefinitions::MAZE_LEN % 2 != 0) {
            return x == midpoint && y == midpoint;
        }

        return  (x == midpoint     && y == midpoint    ) ||
        (x == midpoint - 1 && y == midpoint    ) ||
        (x == midpoint     && y == midpoint - 1) ||
        (x == midpoint - 1 && y == midpoint - 1);
    }
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

