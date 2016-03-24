#include <iostream>
#include <cstdlib>  // atoi

#include "Maze.h"
#include "MazeDefinitions.h"
#include "PathFinder.h"
#include <stack>
#include <queue>
// define infinity
#define INFINITY 10*MazeDefinitions::MAZE_LEN
 
// define instruction lists.
// insn_to_'direction": 'direction'=> position of the next cell in respect to the current cell.
// next cell is in the:             north,      east,       south,          west
// (forwardX,forwardY)=             (0,1)       (1,0)       (0,-1)        (-1,0) 
const MouseMovement insn_to_north[4] = {MoveForward, TurnClockwise, TurnAround, TurnCounterClockwise};
const MouseMovement insn_to_east[4] = {TurnCounterClockwise, MoveForward, TurnClockwise, TurnAround};
const MouseMovement insn_to_south[4] = {TurnAround, TurnCounterClockwise, MoveForward, TurnClockwise};
const MouseMovement insn_to_west[4] = {TurnClockwise, TurnAround, TurnCounterClockwise, MoveForward};

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
        // clearPriority();
    }

    void clearVisits(){
        for (int r = 0; r != MazeDefinitions::MAZE_LEN; r++){
            for (int c = 0; c != MazeDefinitions::MAZE_LEN; c++){
                map[r][c].visited = false;
            }
        }
    }

    // first, clear all visits
    void assign_new_dis(Cell* currCell){
        currCell->distance = 0;
        // currCell->visited = true;
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
        // exit(0);

    }

    MouseMovement setHeadDirection(unsigned x, unsigned y, unsigned nextX, unsigned nextY, Dir funcHeading){
        unsigned forwardX = nextX - x;
        unsigned forwardY = nextY - y;
        int index;
  
        if(forwardX > 0)
            index = 1;
        else if(forwardX < 0)
            index = 3;
        else if(forwardY > 0)
            index = 0;
        else
            index = 2;

        switch(funcHeading){
            case NORTH:
                if(verbose){
                    std::cout << "funcHeading = NORTH\n";
                }
                return insn_to_north[index];
            case SOUTH:
                if(verbose){
                    std::cout << "funcHeading = SOUTH\n";
                }
                return insn_to_south[index];
            case EAST:
                if(verbose){
                    std::cout << "funcHeading = EAST\n";
                }
                return insn_to_east[index];
            case WEST:
                if(verbose){
                    std::cout << "funcHeading = WEST\n";
                }
                return insn_to_west[index];
        }
    }

    // for step one. Does two things:
    // [1] use front, right, left wall status to find min distance.
    // [2] assign return value.(mouse movement)
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
    void find_minDistance_and_nextInsn_II(unsigned x, unsigned y, Dir funcHeading){

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
  
        // In the below three if clauses, we check the distance of the adjacent cell and 
        // set the wall status of the adjacent cell & current cell.

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
                // if(verbose){
                //     std::cout << "II: (" << x - forwardY << "," << y + forwardX << "):visited\n";
                // }
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
    // if there are multiple minimum, return based on priority: front->right->left
    Cell* findMinDistance(unsigned cx, unsigned cy, bool isConstructingRoute = false){
        // Cell* minCell[4] = { &map[cx][cy+1], &map[cx+1][cy], &map[cx][cy-1], &map[cx-1][cy] };
        minMDistance = INFINITY;
        Cell* retCell = NULL;
        if(verbose){
            // std::cout << "currCell=(" << cx << "," << cy << ")\n";
         // check the distance of the grid in the north
        }
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
            
                if(verbose){
                    // std::cout << "map[cx][cy+1].visited= " << map[cx][cy+1].visited << ", dis=" <<  minMDistance << "\n";
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
                if(verbose){
                    // std::cout << "map[cx][cy-1].visited= " << map[cx][cy-1].visited << ", dis=" <<  minMDistance << "\n";
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
                if(verbose){
                    // std::cout << "map[cx+1][cy].visited= " << map[cx+1][cy].visited << ", dis=" <<  minMDistance << "\n";
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
                if(verbose){
                    // std::cout << "map[cx-1][cy].visited= " << map[cx-1][cy].visited << ", dis=" <<  minMDistance << "\n";
                }
        }
        if(verbose){
            std::cout << "final minDistance=" <<  minMDistance << "\n";
            if(retCell != NULL)
                std::cout << "final retCell= (" <<  retCell->cx << "," << retCell->cy << ")" << "\n";
        }
        return retCell;

        // //               north,east,south,west
        // // bool priority_dirs[4] = {false,false,false,false};
        // // check the distance of the grid in the north
        // clearPriority();
        // if(!map[cx][cy].northWall && (map[cx][cy+1].distance <= minMDistance)){
        //     minMDistance = map[cx][cy+1].distance;
        //     priority_dirs[0] = true;
        // }

        // // check the distance of the grid in the east
        // if(!map[cx][cy].eastWall && (map[cx+1][cy].distance <= minMDistance)){
        //     minMDistance = map[cx+1][cy].distance;
        //     if(map[cx+1][cy].distance < minMDistance){
        //         clearPriority();
        //     }
        //     priority_dirs[1] = true;
        // }

        // // check the distance of the grid in the south
        // if(!map[cx][cy].southWall && (map[cx][cy-1].distance <= minMDistance)){
        //     minMDistance = map[cx][cy-1].distance;
        //     if(map[cx][cy-1].distance < minMDistance){
        //         clearPriority();
        //     }
        //     priority_dirs[2] = true; 
        // }

        // // check the distance of the grid in the west
        // if(!map[cx][cy].westWall && (map[cx-1][cy].distance < minMDistance)){
        //     minMDistance = map[cx-1][cy].distance;
        //     if(map[cx-1][cy].distance < minMDistance){
        //         clearPriority();
        //     }
        //     priority_dirs[3] = true;
            
        // }
        // // determine return value
        // int startInd = 0;
        // switch(funcHeading){
        //     case NORTH: startInd = 0;   break;
        //     case EAST: startInd = 1;   break;
        //     case SOUTH: startInd = 2;   break;
        //     case WEST: startInd = 3;   break;
        // }
        // if(verbose){
        //     std::cout << "here\n";
        //     for(int i = 0; i != 4; i++)
        //         std::cout << priority_dirs[i] << ", ";
        //     std::cout << std::endl;
        // }
        // while(1){
        //     if(priority_dirs[startInd%4] == true)
        //         return minCell[startInd%4];
        //     startInd++;
        // }
    }

    void constructRoute(){
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
        //     MouseMovement nextMovement = setHeadDirection(currCell->cx, currCell->cy, nextCell->cx, nextCell->cy, funcHeading);
            // update Cell and funcHeading  
            switch(retval){
                case TurnAround:
                    if(verbose){
                        std::cout << "TurnAround, ";
                    }
                    funcHeading = opposite(funcHeading);
                break;
                case TurnClockwise:
                    if(verbose){
                        std::cout << "TurnClockwise, ";
                    }
                    funcHeading = clockwise(funcHeading);
                break;
                case TurnCounterClockwise:
                    if(verbose){
                        std::cout << "TurnCounterClockwise, ";
                    }
                    funcHeading = counterClockwise(funcHeading);
                break;
                case MoveForward:
                    if(verbose){
                        std::cout << "MoveForward, ";
                    }
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
                break;
                case Wait:
                    if(verbose){
                        std::cout << "Wait, ";
                    }
                break;
            }
            routeSt1.push(retval);
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
   
    void HomeBoundMode(unsigned x, unsigned y){
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

    void FastMode(unsigned x, unsigned y){
        if(routeSt2.empty()){
            retval = Wait;
            return;
        }
        retval = routeSt2.top();
        routeSt1.push(retval);
        routeSt2.pop();
    }
    // rerun floodfill "virtually"
    void virtualRun(){
        // set up initial values:
        Dir virtualheading = NORTH;
        unsigned vx = 0;
        unsigned vy = 0;

        while(map[vx][vy].distance != 0){
            nextMove(vx,vy, virtualheading);
            switch(retval){
                case TurnAround:
                    if(verbose){
                        std::cout << "TurnAround, ";
                    }
                    virtualheading = opposite(virtualheading);
                break;
                case TurnClockwise:
                    if(verbose){
                        std::cout << "TurnClockwise, ";
                    }
                    virtualheading = clockwise(virtualheading);
                break;
                case TurnCounterClockwise:
                    if(verbose){
                        std::cout << "TurnCounterClockwise, ";
                    }
                    virtualheading = counterClockwise(virtualheading);
                break;
                case MoveForward:
                    if(verbose){
                        std::cout << "MoveForward, ";
                    }
                    switch(virtualheading){
                        case NORTH:
                            vy++;
                            break;
                        case SOUTH:
                            vy--;
                            break;
                        case EAST:
                            vx++;
                            break;
                        case WEST:
                            vx--;
                            break;
                    }

                break;
                case Wait:
                    if(verbose){
                        std::cout << "Wait, ";
                    }
                break;
            }
        }
    }

    void nextMove(unsigned vx, unsigned vy, Dir virtualheading){
         ////////////////////////////////////////////////////////////////////////////////////////////////////
        // step 1: Follow Manhattan Distances downwards towards center, noting down any walls as they pass
        ////////////////////////////////////////////////////////////////////////////////////////////////////

        find_minDistance_and_nextInsn_II(vx,vy, virtualheading);
        // if mimMDistance is not changed, then currMDistance is the smallest Mdistance in its neighborhood.
        if(minMDistance != map[vx][vy].distance)
            return;

        if(verbose)
            std::cout << "Step 1 done...";

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        // step 2: Apply floodfill algorithm
        ////////////////////////////////////////////////////////////////////////////////////////////////////

        

        // new stack //Stack of points to be processed (can also use queue)
        std::stack<Cell*> st;

        // push current cell onto stack
        st.push(&map[vx][vy]);
        Cell* curr; 


        while (!st.empty()){
            curr = st.top();
            st.pop();
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
            findMinDistance(cx,cy, true);
            
            
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
        // // push origin
        // routeSt1.push(Wait);
        // Cell* currCell = &map[0][0];
        // Cell* nextCell;
        // Dir funcHeading = NORTH;
        // while(currCell->distance != 0){
        //     nextCell = findMinDistance(currCell->cx, currCell->cy, true);
        //     MouseMovement nextInsn = setHeadDirection(currCell->cx, currCell->cy, nextCell->cx, nextCell->cy, funcHeading);

        //     // if(verbose){
        //     //     std::cout << "\nminMDistance = " << minMDistance << std::endl;
        //     //     std::cout << "(" << currCell->cx << ","<< currCell->cy << ")=" << currCell->distance << "nextInsn= ";
        //     // }
        //     routeSt1.push(nextInsn);
        //     // update Cell and funcHeading
        //     switch(nextInsn){
        //         case TurnAround:
        //             if(verbose){
        //                 std::cout << "TurnAround, ";
        //             }
        //             funcHeading = opposite(funcHeading);
        //         break;
        //         case TurnClockwise:
        //             if(verbose){
        //                 std::cout << "TurnClockwise, ";
        //             }
        //             funcHeading = clockwise(funcHeading);
        //         break;
        //         case TurnCounterClockwise:
        //             if(verbose){
        //                 std::cout << "TurnCounterClockwise, ";
        //             }
        //             funcHeading = counterClockwise(funcHeading);
        //         break;
        //         case MoveForward:
        //             if(verbose){
        //                 std::cout << "MoveForward, ";
        //             }
        //         break;
        //         case Wait:
        //             if(verbose){
        //                 std::cout << "Wait, ";
        //             }
        //         break;
        //     }
        //     currCell = nextCell;
        //     if(verbose){
        //     //     std::cout << "nextCell=(" << currCell->cx << ","<< currCell->cy << ")=" << currCell->distance << "\n";
        //         std::cout << "\n\n";
        //     }
        //     // routeSt1.push(findMinDistance(curr->cx,curr->cy));
        // }
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
                clearVisits();
                assign_new_dis(&map[x][y]);
                // virtualRun();
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
    unsigned maxMDistance;
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
    std::stack<MouseMovement> routeSt1;
    std::stack<MouseMovement> routeSt2;

    // north,east,south,west
    bool priority_dirs[4];

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

