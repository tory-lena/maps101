/* CS 106B - assignment 7
 *
 * Name : Victoria Magdalena Dax
 * TA : Nathan Orttung
 *
 * Extension: bidirectional A* alg. implemented as "aStar_biDir()"
 */

#include "Trailblazer.h"
#include "queue.h"
#include "priorityqueue.h"
#include "vector.h"


using namespace std;

static const double SUFFICIENT_DIFFERENCE = 0.2;

// BFS - can not guarantee path to be optimal
Path breadthFirstSearch(const RoadGraph& graph, RoadNode* start, RoadNode* end) {

    if (start == end){
        start->setColor(Color::GREEN);
        return {start};
    }

    Queue<Path> queue;
    Path curr;
    RoadNode* temp;
    Set<RoadNode*> neigh;
    Set<RoadNode*> seen;

    queue.enqueue({start});

    while (!queue.isEmpty()){
        curr = queue.dequeue();
        temp = curr.pop_back();
        temp->setColor(Color::GREEN);
        curr.push_back(temp);
        neigh = graph.neighborsOf(temp);
        for (RoadNode* v : neigh){
            if (!seen.contains(v)){
                curr.push_back(v);
                if (v==end){ // teminate early
                    for (RoadNode* node : curr){
                        node->setColor(Color::GREEN);
                    }
                    return curr;
                }
                v->setColor(Color::YELLOW);
                queue.enqueue(curr);
                curr.pop_back();
                seen.add(v);
            }
        }
    }
    return {};
}

/* Helper function
 * called by dijkstra() and aStar() and aStar_biDir()
 * can be used with additional heuristic when called with target = end
 * for example.
 */
float get_cost(const RoadGraph& graph, Path path, RoadNode* target = nullptr){
    RoadNode* end = path.pop_back();
    RoadNode* start;
    RoadEdge* road;
    float val = 0;

    if (target != nullptr){
        val += graph.crowFlyDistanceBetween(end, target)/graph.maxRoadSpeed();
    }

    while (!path.isEmpty()){
        start = path.pop_back();
        road = graph.edgeBetween(start, end);
        val += road->cost();
        end = start;
    }

    return val;
}

// Dijstra alg - finds opt path
Path dijkstrasAlgorithm(const RoadGraph& graph, RoadNode* start, RoadNode* end) {

    if (start == end){
        start->setColor(Color::GREEN);
        return {start};
    }

    PriorityQueue<Path> queue;
    Path curr = {start};
    RoadNode* temp;
    Set<RoadNode*> neigh;
    Set<RoadNode*> seen;

    start->setColor(Color::YELLOW);
    queue.enqueue(curr, get_cost(graph, curr));

    while (!queue.isEmpty()){
        curr = queue.dequeue();
        temp = curr.pop_back();
        seen.add(temp);
        temp->setColor(Color::GREEN);
        curr.push_back(temp);
        if (temp==end){
            return curr;
        }
        neigh = graph.neighborsOf(temp);
        for (RoadNode* v : neigh){
            curr.push_back(v);
            if (!seen.contains(v)){
                v->setColor(Color::YELLOW);
                queue.enqueue(curr, get_cost(graph, curr));
            }
            curr.pop_back();
        }
    }
    return {};
}

// A* alg - Dijkstra with asTheCrowFlies Heuristic
Path aStar(const RoadGraph& graph, RoadNode* start, RoadNode* end) {

    if (start == end){
        start->setColor(Color::GREEN);
        return {start};
    }

    PriorityQueue<Path> queue;
    Path curr = {start};
    RoadNode* temp;
    Set<RoadNode*> neigh;
    Set<RoadNode*> seen;

    start->setColor(Color::YELLOW);
    queue.enqueue(curr, get_cost(graph, curr, end));

    while (!queue.isEmpty()){
        curr = queue.dequeue();
        temp = curr.pop_back();
        seen.add(temp);
        temp->setColor(Color::GREEN);
        curr.push_back(temp);
        if (temp==end){
            return curr;
        }
        neigh = graph.neighborsOf(temp);
        for (RoadNode* v : neigh){
            curr.push_back(v);
            if (!seen.contains(v)){
                v->setColor(Color::YELLOW);
                queue.enqueue(curr, get_cost(graph, curr, end));
            }

            curr.pop_back();
        }
    }

    return {};
}

/* The Idea is that two small trees are "cheaper" than one large one.
 * Thus a bidirectional search explores from either start and from end
 * WHEN/If both trees touch we have found a path
 */

float find_closest(const RoadGraph& graph, RoadNode* v, Set<RoadNode*> seen){
    float cost = 1e30;
    float dist = 0;
    for (RoadNode* node : seen){
        dist = graph.crowFlyDistanceBetween(v, node);
        if (dist<cost) cost = dist;
    }
    return cost / graph.maxRoadSpeed();
}

// Helper function
// called by aStar_biDir()
// coded because <vector>.reverse() function wouldn't compile
Path reverse(Path path){
    Path rev;
    while (!path.isEmpty()){
        rev.push_back(path.pop_back());
    }
    return rev;
}

// Helper function
// called by aStar_biDir() - gets the other part of the path
Path get_second_half(PriorityQueue<Path> queue, RoadNode* node){
    Path curr;
    while (!queue.isEmpty()){
        curr = queue.dequeue();
        if (curr.contains(node)){
            Path sol;
            for (RoadNode* v : curr){
                if (v==node) break;
                sol.push_back(v);
            }
            return sol;
        }
    }
    return {};
}

/* Bidirectional A*
 * This function is only guarantee to work for non-directional graphs
 * Else we would have to substitute neighborsOf() with neighborsTo() in the
 * reverse path search part
 */
Path aStar_biDir(const RoadGraph& graph, RoadNode* start, RoadNode* end){
    if (start == end){
        start->setColor(Color::GREEN);
        return {start};
    }

    PriorityQueue<Path> queue_from;
    Set<RoadNode*> seen_from;
    Path curr = {start};
    start->setColor(Color::YELLOW);
    queue_from.enqueue(curr, get_cost(graph, curr, end));

    RoadNode* temp;
    Set<RoadNode*> neigh;

    PriorityQueue<Path> queue_to;
    curr = {end};
    Set<RoadNode*> seen_to;
    end->setColor(Color::YELLOW);
    queue_to.enqueue(curr, get_cost(graph, curr, start));
    seen_to.add(end);

    while (!queue_from.isEmpty() && !queue_to.isEmpty()){
        curr = queue_from.dequeue();
        temp = curr.pop_back();
        seen_from.add(temp);
        temp->setColor(Color::GREEN);
        curr.push_back(temp);
        if (seen_to.contains(temp)){
            return curr+reverse(get_second_half(queue_to, temp));
        }
        neigh = graph.neighborsOf(temp);
        for (RoadNode* v : neigh){
            curr.push_back(v);
            if (!seen_from.contains(v) && !seen_to.contains(v)){
                v->setColor(Color::YELLOW);
            }
            queue_from.enqueue(curr, get_cost(graph, curr, end));//+find_closest(graph, v, seen_to));
            curr.pop_back();
        }

        curr = queue_to.dequeue();
        temp = curr.pop_back();
        seen_to.add(temp);
        temp->setColor(Color::GREEN);
        curr.push_back(temp);
        if (seen_from.contains(temp)){
            return get_second_half(queue_from, temp)+reverse(curr);
        }
        neigh = graph.neighborsOf(temp);
        for (RoadNode* v : neigh){
            curr.push_back(v);
            if (!seen_to.contains(v) && !seen_from.contains(v)){
                v->setColor(Color::YELLOW);
            }
            queue_to.enqueue(curr, get_cost(graph, curr, start));//+find_closest(graph, v, seen_from));
            curr.pop_back();
        }
    }
    return {};
}

// helper fucntion - A* (without the edge "edge")
Path aStar(const RoadGraph& graph, RoadNode* start, RoadNode* end, RoadEdge* edge) {

    if (start == end){
        start->setColor(Color::GREEN);
        return {start};
    }

    PriorityQueue<Path> queue;
    Path curr = {start};
    RoadNode* temp;
    Set<RoadNode*> neigh;
    Set<RoadNode*> seen;

    start->setColor(Color::YELLOW);
    queue.enqueue(curr, get_cost(graph, curr, end));

    while (!queue.isEmpty()){
        curr = queue.dequeue();
        temp = curr.pop_back();
        seen.add(temp);
        temp->setColor(Color::GREEN);
        curr.push_back(temp);
        if (temp==end){
            return curr;
        }
        neigh = graph.neighborsOf(temp);
        if (temp == edge->from()){
            neigh.remove(edge->to());
        }
        for (RoadNode* v : neigh){
            curr.push_back(v);
            if (!seen.contains(v)){
                v->setColor(Color::YELLOW);
                queue.enqueue(curr, get_cost(graph, curr, end));
            }
            curr.pop_back();
        }
    }
    return {};
}

/* Alternative Route
 * Drop one edge of the fastest route computed with A*, recompute a "fastest"
 * under these conditions. If it has sufficient different, and is "cheaper" than
 * current alternative, substitute.
 */
Path alternativeRoute(const RoadGraph& graph, RoadNode* start, RoadNode* end) {
    // get shortest path
    Path shortest = aStar(graph, start, end);
    if (shortest.size()<=1){ // no path found or start=end
        return shortest;
    }
    int L = shortest.size();

    Path my_alt = {};
    float count = 0.;
    float my_cost = 1e30;
    // if an edge does not exist - two cities are no longer direct neighbors
    for (int  i=0; i<shortest.size()-1; i++){
        RoadNode* curr = shortest[i];
        RoadNode* next = shortest[i+1];
        RoadEdge* edge =  graph.edgeBetween(curr, next);
        Path option = aStar(graph, start, end, edge);

        // valid?
        count = 0;
        for (RoadNode* curr : option){
            if (!shortest.contains(curr)) count+=1.;
        }
        if (count/L > SUFFICIENT_DIFFERENCE && get_cost(graph, option) < my_cost){
            my_cost = get_cost(graph, option);
            my_alt = option;
        }
    }

    return my_alt;
}
