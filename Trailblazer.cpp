//This is the cpp file that implements the four search algorithms for finding paths.

#include "Trailblazer.h"
#include "queue.h"
#include "set.h"
#include "priorityqueue.h"
using namespace std;

static const double SUFFICIENT_DIFFERENCE = 0.2;
Path alternativeHelper(const RoadGraph &graph, RoadNode *start, RoadNode *end, Set<RoadNode *> dontVisit);

Path breadthFirstSearch(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
    Queue<Path> todo; // Paths yet to check
    Path p;
    Set<RoadNode*> visitedNodes; //  Set of nodes that have been visited already
    p.add(start);
    todo.add(p); // Adding first path (which contains only "start" element)
    while(!todo.isEmpty() && !visitedNodes.contains(end)){ // Stops looping if end has been visited or queue is empty
        Path currPath = todo.remove();
        RoadNode* currNode = currPath[currPath.size()-1]; // currNode is last node of currPath
        visitedNodes.add(currNode);
        currNode->setColor(Color::GREEN); // Setting visited node colour to green
        if (currNode == end){
            return currPath;
        } else {
            Set<RoadNode*> neighbors =  graph.neighborsOf(currNode); // Set of all neighbours of currNode
            for (RoadNode* r: neighbors){ // for all neighbours
                if (!visitedNodes.contains(r)){
                    r->setColor(Color::YELLOW); // yellow colour for nodes that will be visited later
                    currPath.add(r);
                    todo.enqueue(currPath); // pushing to end of queue
                    currPath.remove(currPath.size()-1); // restoring currPath by deleting its last element
                }
            }
        }
    }
    p.clear(); // only runs if no path found, returns empty path
    return p;
}

Path dijkstrasAlgorithm(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
    PriorityQueue<Path> todo;
    Path p;
    Set<RoadNode*> visitedNodes;
    p.add(start);
    int pathLength = 0; // this is the initial priority
    todo.enqueue(p, pathLength);
    while(!todo.isEmpty() && !visitedNodes.contains(end)){
        Path currPath = todo.dequeue();
        RoadNode* currNode = currPath[currPath.size()-1]; // currNode is last element of currPath
        visitedNodes.add(currNode);
        currNode->setColor(Color::GREEN);
        if (currNode == end){
            return currPath;
        } else {
            Set<RoadNode*> neighbors =  graph.neighborsOf(currNode);
            for (RoadNode* r: neighbors){
                if (!visitedNodes.contains(r)){
                    r->setColor(Color::YELLOW);
                    currPath.add(r);
                    for (int i =0; i<currPath.size()-1; i++){ // calculates pathlength for the path
                        RoadEdge* edge = graph.edgeBetween(currPath[i], currPath[i+1]);
                        pathLength+=edge->cost(); // pathLength is increased by cost of edge between ith and (i+1)th node
                    }
                    todo.enqueue(currPath, pathLength);
                    currPath.remove(currPath.size()-1); // resetting currPath and pathLength
                    pathLength = 0;
                }
            }
        }
    }
    p.clear(); // runs only if no solution found, returns empty path
    return p;
}

Path aStar(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
    Set<RoadNode*> dontVisit;
    dontVisit.clear();
    return alternativeHelper(graph, start, end, dontVisit);
}

Path alternativeRoute(const RoadGraph& graph, RoadNode* start, RoadNode* end) {

    Set<RoadNode*> dontVisit; // empty dontVisit set
    Path bestPath = alternativeHelper(graph,start,end,dontVisit); // best path found by sending empty dontVisit set

    Set<RoadEdge*> edgeSet; // set of all edges in best path
    for (int i =0; i<bestPath.size()-1; i++){
        edgeSet.add(graph.edgeBetween(bestPath[i], bestPath[i+1])); // adding edges one by one
    }
    Vector<Path> alternatePaths; // Vector of all possible alternate paths
    for (RoadEdge* r : edgeSet){
        RoadNode* startNode = r->from();
        RoadNode* endNode = r->to();
        dontVisit.add(startNode);
        dontVisit.add(endNode); // for each edge, we add its starting & ending node to dontVisit
        alternatePaths.add(alternativeHelper(graph, start, end, dontVisit)); // best path without using this edge
        dontVisit.clear(); // resetting dontVisit for nect iteration
    }
    // Now, alternatePaths has all the possible best paths which exclude one of all the edges of the normal best path
    // We have to find the best out of all these. Thus, we must find the shortest pathlength path

    int minPathLengthPos, pathLength=0, shortestPathLength;
    for (int i =0; i<alternatePaths.size(); i++){ // for each path
        for (int  j=0; j<alternatePaths[i].size()-1; j++){ // calculating pathlength
            RoadEdge* edge = graph.edgeBetween((alternatePaths[i])[j], (alternatePaths[i])[j+1]);
            pathLength+= edge->cost();
        }
        if (i==0){ // initially, we assume thatt first path's pathlength is shortest
            shortestPathLength = pathLength;
            minPathLengthPos = 0;
        } else { // if i!=0, we compare new path's length with previous shortest
            if (pathLength<shortestPathLength){
                shortestPathLength = pathLength;
                minPathLengthPos = i;
            }
        }
    }
    return alternatePaths[minPathLengthPos]; // returns shortest length path from alternatePaths

}

Path alternativeHelper(const RoadGraph& graph, RoadNode* start, RoadNode* end, Set<RoadNode*> dontVisit){
    PriorityQueue<Path> todo; // Same General code as in dijkstra
    Path p, currPath;
    Set<RoadNode*> visitedNodes;
    p.add(start);
    int pathLength = 0;
    todo.enqueue(p, pathLength);
    while(!todo.isEmpty() && !visitedNodes.contains(end)){
        currPath = todo.dequeue();
        RoadNode* currNode = currPath[currPath.size()-1];
        visitedNodes.add(currNode);
        currNode->setColor(Color::GREEN);
        if (currNode == end){
            return currPath;
        } else {
            Set<RoadNode*> neighbors =  graph.neighborsOf(currNode);
            for (RoadNode* r: neighbors){
                if (!visitedNodes.contains(r) && !dontVisit.contains(r)){ //ONLY DIFF: doesnt visit nodes in set "dontVisit"
                    r->setColor(Color::YELLOW);
                    currPath.add(r);
                    for (int i =0; i<currPath.size()-1; i++){
                        RoadEdge* edge = graph.edgeBetween(currPath[i], currPath[i+1]);
                        pathLength+=edge->cost();
                    } // heuristic function is added to pathlength below to get final priority
                    double priority = pathLength + graph.crowFlyDistanceBetween(currPath[currPath.size()-1], end)/graph.maxRoadSpeed();
                    todo.enqueue(currPath, priority);
                    currPath.remove(currPath.size()-1); // resetting currPath and pathLength
                    pathLength = 0;
                }
            }
        }
    }
    p.clear(); // runs only if no path found, returns empty path
    return p;
}

