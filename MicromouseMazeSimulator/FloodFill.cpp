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
    enum Section{
        SECTION_LOWER_LEFT, SECTION_UPPER_LEFT, SECTION_LOWER_RIGHT, SECTION_UPPER_RIGHT, SECTION_INVALID
    };

    struct Coord{
        Coord(){
            northWall = true;
            southWall = true;
            eastWall = true;
            westWall = true;
            distance = 16;
            check = false;
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
        bool check;
        unsigned distance;
        unsigned cx, cy;
    };

    FloodFill(bool shouldPause = false, bool shouldPrint = false) : pause(shouldPause), verbose(shouldPrint) {
        shouldGoForward = false;
        visitedStart = false;

        // construct map
        for (int r = 0; r != MazeDefinitions::MAZE_LEN; r++){
            for (int c = 0; c != MazeDefinitions::MAZE_LEN; c++){
                map[r][c].distance=getManDistance(r,c);
                map[r][c].setXY(r,c);
            }
        }
    }
    void clearCheck(){
        for (int r = 0; r != MazeDefinitions::MAZE_LEN; r++){
            for (int c = 0; c != MazeDefinitions::MAZE_LEN; c++){
                map[r][c].check = false;
            }
        } 
    }

    void find_minDistance_and_nextInsn(unsigned x, unsigned y,bool checkBack = false){
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
  
        // check the Mdistance of the grid at the front
        if(!frontWall){
            map[x][y].setWall(currHeading);
            map[x+ forwardX][y+ forwardY].setWall(opposite(currHeading));
            
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


    unsigned getManDistance(unsigned x, unsigned y){
        // error checking
        if (x >= MazeDefinitions::MAZE_LEN || y >= MazeDefinitions::MAZE_LEN){
            // std::cout << "Error: invalid (x,y) coordinates." << std::endl;
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

    Section getSection(unsigned x, unsigned y){
        // error checking
        if (x >= MazeDefinitions::MAZE_LEN || y >= MazeDefinitions::MAZE_LEN){
            // std::cout << "Error: invalid (x,y) coordinates." << std::endl;
            return SECTION_INVALID;
        }
        const unsigned mid = MazeDefinitions::MAZE_LEN / 2;
        // lower left section
        if (x < mid && y < mid)
            return SECTION_LOWER_LEFT;
        // upper left section
        if (x < mid)
            return SECTION_UPPER_LEFT;
        // lower right section
        if (y < mid)
            return SECTION_LOWER_RIGHT;
        // upper right section
        return SECTION_UPPER_RIGHT;
    }


    MouseMovement nextMovement(unsigned x, unsigned y, const Maze &maze) {
        // const bool NorthWall = maze.wallOnNorth();
        // const bool SouthWall = maze.wallOnSouth();
        // const bool EastWall = maze.wallOnEast();
        // const bool WestWall = maze.wallOnWest();

        frontWall = maze.wallInFront();
        leftWall  = maze.wallOnLeft();
        rightWall = maze.wallOnRight();

        currMDistance = map[x][y].distance;//getManDistance(x,y);
        // currSection = getSection(x,y);
        currHeading = maze.getHeading();

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
            std::cout << "Found center! Good enough for the demo, won't try to get back." << std::endl;
            return Finish;
        }

        // If we hit the start of the maze a second time, then
        // we couldn't find the center and never will...
        if(x == 0 && y == 0) {
            if(visitedStart) {
                std::cout << "Unable to find center, giving up." << std::endl;
                return Finish;
            } else {
                visitedStart = true;
            }
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        // step 1: Follow Manhattan Distances downwards towards center, noting down any walls as they pass
        ////////////////////////////////////////////////////////////////////////////////////////////////////

        find_minDistance_and_nextInsn(x,y);
        // if mimMDistance is not changed, then currMDistance is the smallest Mdistance in its neighborhood.
        if(minMDistance != currMDistance)
            return retval;

            
        std::cout << "step 1 done...";
        // return Finish;

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        // step 2: 
        ////////////////////////////////////////////////////////////////////////////////////////////////////

        // define infinity
        const unsigned INFINITY = 100+MazeDefinitions::MAZE_LEN;


        // construct map

        // new stack //Stack of points to be processed (can also use queue)
        std::stack<Coord*> st;

        // push current cell onto stack
        st.push(&map[x][y]);
        Coord* curr;        
        clearCheck();

        while (!st.empty()){
            curr = st.top();
            curr->check = true;
            st.pop();
            unsigned cx = curr->cx;
            unsigned cy = curr->cy;
            if(verbose){
                std::cout << "curr=[" << cx << "][" << cy << "],dis=" <<curr->distance;
            }
            if (curr->distance == 0)
                continue; //don’t want to process the end goal
            minMDistance = INFINITY;

        //     for each neighboring cell of cur:
      
            // check the Mdistance of the grid in the north
            if(!map[cx][cy].northWall && (map[cx][cy+1].distance <= minMDistance)){
                minMDistance = map[cx][cy+1].distance;
            }
            // check the Mdistance of the grid in the south
            if(!map[cx][cy].southWall && (map[cx][cy-1].distance <= minMDistance)){
                minMDistance = map[cx][cy-1].distance;
            }
            // check the Mdistance of the grid in the east
            if(!map[cx][cy].eastWall && (map[cx+1][cy].distance <= minMDistance)){
                minMDistance = map[cx+1][cy].distance;
            }
            // check the Mdistance of the grid in the west
            if(!map[cx][cy].westWall && (map[cx-1][cy].distance < minMDistance)){
                minMDistance = map[cx-1][cy].distance;
            }

            if(minMDistance == INFINITY)
                continue;
            // if (minMDistance == curr->distance + 1) //nothing was updated, move on
            //     continue;
            curr->setDistance(minMDistance + 1); //new minimum distance
            
            if(verbose)
                std::cout << "{calcMin=" << minMDistance << ", set [" << cx << "][" << cy << "]= " << curr->distance << ",push ";

        //     push every connected neighbor onto stack
            if(!map[cx][cy].northWall && map[cx][cy+1].check){
                st.push(&map[cx][cy+1]);
                if(verbose)
                    std::cout << "[" << cx << "][" << cy+1<< "], ";
            }
            if(!map[cx][cy].southWall && map[cx][cy-1].check){
                st.push(&map[cx][cy-1]);
                if(verbose)
                    std::cout << "[" << cx << "][" << cy-1 << "], ";
            }
            if(!map[cx][cy].eastWall && map[cx+1][cy].check){
                st.push(&map[cx+1][cy]);
                if(verbose)
                    std::cout << "[" << cx+1 << "][" << cy << "], ";
            }
            if(!map[cx][cy].westWall && map[cx-1][cy].check){
                st.push(&map[cx-1][cy]);
                if(verbose)
                    std::cout << "[" << cx-1 << "][" << cy << "], ";
            }
            if(verbose)
                std::cout << "} ";
        }
        return TurnAround;
            
    }

protected:

    // debugging purpose
    bool verbose; 

    // Helps us determine that we should go forward if we have just turned left.
    bool shouldGoForward;

    // Helps us determine if we've made a loop around the maze without finding the center.
    bool visitedStart;

    // Indicates we should pause before moving to next cell.
    // Useful for command line usage.
    const bool pause;

    // record the current Manhattan distance.
    unsigned currMDistance;
    Section currSection;
    Dir currHeading;

    unsigned minMDistance;
    MouseMovement retval;

    // keep track of current neighboring walls status
    bool frontWall;
    bool leftWall;
    bool rightWall;

    // hold mDistance
    Coord map[MazeDefinitions::MAZE_LEN][MazeDefinitions::MAZE_LEN];


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
        } else {
            std::cout << "Usage: " << argv[0] << " [-m N] [-p]" << std::endl;
            std::cout << "\t-m N will load the maze corresponding to N, or 0 if invalid N or missing option" << std::endl;
            std::cout << "\t-p will wait for a newline in between cell traversals" << std::endl;
            return -1;
        }
    }

    FloodFill floodfill(pause, verbose);
    Maze maze(mazeName, &floodfill);
    std::cout << maze.draw(5) << std::endl << std::endl;

    maze.start();
}

