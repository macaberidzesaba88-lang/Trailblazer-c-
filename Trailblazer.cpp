#include "Trailblazer.h"
#include "TrailblazerGraphics.h"
#include "TrailblazerTypes.h"
#include "TrailblazerPQueue.h"
#include "vector.h"
#include "random.h"
#include "map.h" 
#include <algorithm>
#include "error.h"
#include <climits>
using namespace std;

/* Function: shortestPath
 *
 * Finds the shortest path between the locations given by start and end in the
 * specified world.	 The cost of moving from one edge to the next is specified
 * by the given cost function.	The resulting path is then returned as a
 * Vector<Loc> containing the locations to visit in the order in which they
 * would be visited.	If no path is found, this function should report an
 * error.
 *
 * In Part Two of this assignment, you will need to add an additional parameter
 * to this function that represents the heuristic to use while performing the
 * search.  Make sure to update both this implementation prototype and the
 * function prototype in Trailblazer.h.
 */
 //time copmlexity for dijkastra is V log V cause we traver try V times and pq needs log V for each dequemin method
 //same for A*
 //space cmoplexity o v (storing colors dist pq parent)
Vector<Loc>
shortestPath(Loc start,
    Loc end,
    Grid<double>& world,
    double costFn(Loc from, Loc to, Grid<double>& world), double heuristic(Loc start, Loc end, Grid<double>& world)) {
    int n = world.numRows();
    int m = world.numCols();
    Grid<Color>colors(n, m);
    TrailblazerPQueue<Loc> pq;//i use this cause we need each time cheapest node
    Grid<double>dist(n, m);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            colors[i][j] = GRAY;
            dist[i][j] = INT_MAX;
        }
    }
    colors[start.row][start.col] = YELLOW;
    colorCell(world, start, YELLOW);//make yellow, its rule for graphics
    dist[start.row][start.col] = 0;
    pq.enqueue(start, heuristic(start, end, world));;
    bool pathexsists = false;
    Map <Loc, Loc>parent;//best structure for building path is Map which on each time will have parent child key value
    while (!pq.isEmpty()) {
        //cell with smallest distance , from start to this node ,tehre cant be any cheaper path,cause all other paths
        //would include more expensive or equal paths  plus at least edges cost of them -> this cell
        Loc currentmin = pq.dequeueMin();
        colorCell(world, currentmin, GREEN);
        double distance = dist[currentmin.row][currentmin.col];
        colors[currentmin.row][currentmin.col] = GREEN;
        if (currentmin == end) {
            pathexsists = true;//end loop here and build path afterwards
            break;
        }

        if (currentmin == end) {
            break;//end loop here
        }
        for (int ii = -1; ii <= 1; ii++) {
            for (int jj = -1; jj <= 1; jj++) {
                if (ii == 0 && jj == 0) {
                    continue; //skip same node, in this case node doesnt change
                }
                int r = currentmin.row + ii;
                int c = currentmin.col + jj;//generate all 8 directions and continue on legit ones
                if (r < 0 || c < 0 || r >= n || c >= m) {
                    continue; //if its out of bounds skip it
                }
                Loc v = makeLoc(r, c);
                double newDist = distance + costFn(currentmin, v, world);
                double priority = newDist + heuristic(v, end, world);
                if (colors[r][c] == GRAY) {//we found a first path to neighbour and remember it
                    colorCell(world, v, YELLOW);//if its 
                    colors[r][c] = YELLOW;
                    dist[r][c] = newDist;
                    parent[v] = currentmin;
                    pq.enqueue(v, priority);
                }
                else if (colors[r][c] == YELLOW && dist[r][c] > newDist) {
                    dist[r][c] = newDist;//neighbor is in queue already but this path is cheaper
                    parent[v] = currentmin;
                    pq.decreaseKey(v, priority);//pq needs update too, even though its not
                }//written in pseudocode of assingment 
            }
        }
    }
    if (!pathexsists) {
        error("no path exsists");
    }

    Vector<Loc>path;
    Loc cur = end;
    while (cur != start) {//while we not go back into the future, continue loop/traversing
        path.push_back(cur);
        cur = parent[cur];

    }
    path.push_back(start);//loop ends when finds cur but never adds it to path, tricky edge case
    std::reverse(path.begin(), path.end());
    return path;
}
Loc rootfind(Map<Loc, Loc>& parent, Loc nodari) {//time cmomplexity Vlog V because it may traverse in depth
    if (parent[nodari] == nodari) {//find root with recursion
        return nodari;
    }
    return rootfind(parent, parent[nodari]);

}
Set<Edge> createMaze(int numRows, int numCols) {
    //this funciton is union find with map basically
    //time copmlexity is E log E was generating all edges
    //for processing edges worst time complexity is Elog E
    //calling rootfind E times gives us creatyeMazes worst time complexity to be E* Vlog V
    //space is O(V+E)
    Map<Loc, Loc>parent;//node points to its parent, i chose it cause its good parent child pair and also good time efficiency
    TrailblazerPQueue<Edge> pq;//kruskals algorithm is greedy, so for time efficiency we need this DS
    Set<Edge> mzismaze;//i chose it cause it handles duplicaes well and its basically return type
    for (int r = 0; r < numRows; r++) {//at every node is boss itself
        for (int c = 0; c < numCols; c++) {
            Loc current = makeLoc(r, c);
            parent[current] = current;
            if (c + 1 < numCols) {//avoid duplicates bygoing only right and down cause its symmetry
                pq.enqueue(makeEdge(current, makeLoc(r, c + 1)), randomReal(0, 1));
            }
            if (r + 1 < numRows) {
                pq.enqueue(makeEdge(current, makeLoc(r + 1, c)), randomReal(0, 1));
            }

        }
    }
    //traversing/processing edges
    while (!pq.isEmpty() && mzismaze.size() < numRows * numCols - 1) {//we limit maze by how many edges it needs
        Edge x = pq.dequeueMin();//so loop must be while size also is less than that mathematically
        Loc y1 = rootfind(parent, x.start);
        Loc y2 = rootfind(parent, x.end);
        if (y1 != y2) {
            parent[y1] = y2;
            mzismaze.add(x);
        }
    }
    return mzismaze;
}