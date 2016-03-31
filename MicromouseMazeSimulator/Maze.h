#ifndef Maze_h
#define Maze_h

#include <string>

#include "BitVector256.h"
#include "MazeDefinitions.h"
#include "Dir.h"
#include "PathFinder.h"


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


class Maze {
protected:
    BitVector256 wallNS;
    BitVector256 wallEW;
    Dir heading;
    PathFinder *pathFinder;
    unsigned mouseX;
    unsigned mouseY;

    bool isOpen(unsigned x, unsigned y, Dir d) const;
    void setOpen(unsigned x, unsigned y, Dir d);

    void moveForward();
    void moveBackward();

    inline void turnClockwise() {
        heading = clockwise(heading);
    }

    inline void turnCounterClockwise() {
        heading = counterClockwise(heading);
    }

    inline void turnAround() {
        heading = opposite(heading);
    }

public:
    Maze(MazeDefinitions::MazeEncodingName name, PathFinder *pathFinder);

    inline bool wallInFront() const {
        return !isOpen(mouseX, mouseY, heading);
    }

    inline bool wallOnLeft() const {
        return !isOpen(mouseX, mouseY, counterClockwise(heading));
    }

    inline bool wallOnRight() const {
        return !isOpen(mouseX, mouseY, clockwise(heading));
    }
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

    // // // for floodfill detection:
    // inline Dir getHeading() const {
    //     return heading;
    // }
    /**
     * Start running the mouse through the maze.
     * Terminates when the PathFinder's nextMovement method returns MouseMovement::Finish.
     */
    void start();

    /**
     * This function draws the maze using ASCII characters.
     *
     * Queries the underlying PathFinder for additional maze info
     * and incorporates it in the maze rendering.
     * @param infoLen: specifies the max characters of info to be drawn. If no info is supplied, blank spaces will be inserted.
     * @return string of rendered maze
     */
    std::string draw(const size_t infoLen = 4) const;
};

#endif
