/*!	\file shortest.cpp 
*
*   \b Author: Joseph Workoff\n
*   \b Filename: shortest.cpp\n
*   \b Purpose: Apply Dijkstra's algorithm to a graph. \n
*   \n
*	This application accepts an input file containing an adjacency list of a weighted directed graph.
*   It will output the shortest paths from the first vertex to every other vertex, and the time it took to calculate this. 
*   
*/

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

#include <queue>
#include <vector>
#include <map>
#include <chrono>
#include <climits> //for INT_MAX


using namespace std;


/*!
*   \struct Node
*   \brief Represents a node adjacency.
*/
struct node{
    int number, weight;
};


/*!
*   \struct compareWeight
*   \brief Comparator based on node's weights, Min - Max.
*/
struct compareWeight{ 
    bool operator()(node const& n1, node const& n2){
        return n1.weight > n2.weight; 
    }
}; 



/*!
*	\param graph - const MultiMap<int,node> : MultiMap containing an adjacency list, formatted <edge source, {destination, cost}>
*	\param numNodes - const int : Number of vertexes in the graph.
*	\param dist - vector<int> : vector to store each node's smallest cost.
*	\param prev - vector<int> : vector to store each node's penultimate node.
*	\brief Finds the shortest paths from 1 to each other node.
*   
*   \par Description
*	dijkstra() applies dijkstra's algorithm to the input adjacency list.
*   djikstra() creates an STL priority queue of nodes to track the node with the current smallest cost.
*	This node is then used to update the costs of each of its neighbors.
*	If the cost of a path to the neighbor through this node is less than the neighbor's current smallest path, the neighbor's cost is updated and pushed on the queue.
*   This continues until the queue is empty, meaning each node has an updated cost.
*
*	\return None.
*/
void dijkstra(const multimap<int, node> graph, const int numNodes, vector<int> &dist, vector<int> &prev);

/*!
*	\param graph - const MultiMap<int,node> : MultiMap containing an adjacency list, formatted <edge source, {destination, cost}>
*	\brief Prints an adjacency list.
*   
*   \par Description
*	printGraph() prints a formatted adjacency list version of the inputed graph. 
*   
*	\return None.
*/
void printGraph(multimap<int, node> graph);

/*!
*	\param prev - const vector<int> : vector containing the penultimate vertex in the shortest path for each node.
*	\param dist - const vector<int> : vector containing the cost for the shortest path to each node.
*	\brief Prints the shortest paths and their costs.
*   
*   \par Description
*	printPaths() outputs the shortest paths from node 1 to each other node, and each path's cost.
*	It finds the path by tracing through each node's previous nodes back to node 1.
*
*	\return None.
*/
void printPaths(const vector<int> prev, const vector<int> dist);

/*!
*	\param graph - const MultiMap<int,node> : Pointer to MultiMap containing an adjacency list, formatted <edge source, {destination, cost}>
*	\param argc - int : Number of command line arguments
*	\param argv - char* array : Command line arguments
*	\brief Inputs the graph from a file.
*   
*   \par Description
*	readGraph() will prompt the user for a file name if one was not specified in the command line. 
*   It will read in the information about each of the graph's edge's source and destination vertices and their costs.
*   This information is stored in the passed in multimap. 
*
*	\return int numNodes - Number of nodes in graph.
*/
int readGraph(multimap<int, node>* graph, int argc, char *argv[]);


/*!
*	\param argc - int : Number of command line arguments
*	\param argv - char* array : Command line arguments
*	\brief Main routine
*   
*   \par Description
*	main() instantiates the graph multimap and calls readGraph() with it. It will track the time that readGraph() runs and output it. 
*   main() then calls printGraph() to output the adjacency list. 
*   main() then calls dijkstra() to find the shortest path from node 1 to each node. It will track the time this takes and output it.
*   If supplied with a file name is argument 3, main will write the time taken to the file. 
*
*	\return none.
*/
int main(int argc, char *argv[]){
    using namespace std::chrono;

    multimap<int, node> graph;

    auto readBegin = high_resolution_clock::now();
    int numNodes = readGraph(&graph, argc, argv);
    auto readEnd = high_resolution_clock::now();
    auto readTicks = duration_cast<microseconds>(readEnd - readBegin);
    
    // printGraph(graph);

    vector<int> dist(numNodes, INT_MAX); 
    vector<int> prev(numNodes, 1);

    auto algoBegin = high_resolution_clock::now();
    dijkstra(graph, numNodes, dist, prev);
    auto algoEnd = high_resolution_clock::now();
    auto algoTicks = chrono::duration_cast<microseconds>(algoEnd - algoBegin);

    cout << "Reading from file completed in " << readTicks.count() << " ms." << endl;
    cout << "Finding all shortest paths completed in " << algoTicks.count() << " ms." << endl;

    if (argc == 3){
        ofstream resfile;
        resfile.open(argv[2], std::ofstream::out | std::ofstream::app);

        resfile << "," << algoTicks.count();
        resfile.close();
    }
    
    printPaths(prev, dist);
}


void dijkstra(const multimap<int, node> graph, const int numNodes, vector<int> &dist, vector<int> &prev){
    struct node n;
    int cost;
    dist[0] = 0;

    priority_queue<node, vector<node>, compareWeight> prio;

    //push first node
    prio.push(node {1, 0});

    //loop until queue is empty.
    //algorithm is complete once the queue is empty.
    while (!prio.empty()){

        //remove the lowest cost node
        n = prio.top();
        prio.pop();

        //update each neighbor - each entry with the same key
        auto n1 = graph.lower_bound(n.number);
        auto n2 = graph.upper_bound(n.number);

        for (; n1 != n2; n1++){
            cost = dist[(*n1).first - 1] + (*n1).second.weight; //calc cost of path that includes this node
            
            //check if new path is shorter
            if (cost < dist[(*n1).second.number - 1]){ 
                dist[(*n1).second.number - 1] = cost; //update cost
                prev[(*n1).second.number - 1] = n.number; //update prev
                prio.push(node {(*n1).second.number, dist[(*n1).second.number - 1]}); //push node onto queue
            }
        }

        // all neighbors updated
    } //end while
} //end dijkstra


void printGraph(multimap<int, node> graph){
    int n = 0;
    char str[10]; //buffer
    multimap<int,node>::iterator i = graph.begin();

    cout << "Analyzing this graph:" << endl << endl;

    cout << endl << "Node | Neighbor:Cost |" << endl;

    for (i; i != graph.end(); i++){ //loop through every edge

        if (i->first != n){ //check if same or new node
            n = i->first; //update
            cout << endl; //end old node's line
            cout << right << setw(4) << i->first << " | "; //start new node's line
        }

        sprintf(str, "%d:%d",i->second.number, i->second.weight); //format edge
        cout << left << setw(6) << str << "| "; //print edge
    }
    cout << endl << endl;
} //end printGraph


void printPaths(const vector<int> prev, const vector<int> dist){
    int p;
    string path = "";
    char buf[10];
    std::string::iterator it;

    cout << endl << "Found these paths:" << endl << endl;

    cout << "src:dst | cost | path" << endl << endl;

    for (int i = 2; i <= dist.size(); i++){

        sprintf(buf, "1:%d", i);
        cout << setw(7) << right << buf << " | ";
        cout << setw(4) << right << dist[i - 1] << " | "; 

        if (dist[i] == INT_MAX){
            cout << "No Path Found." << endl;
        }
        else{

            //trace through previous nodes until reaching node 1
            p = i;
            while (p != 1){
                path = to_string(p) + " " + path;
                p = prev[p - 1]; 
            }
            path = "1 " + path;

            cout << path << endl;
                       
        }
        path.clear();
    } //end for

} //end printPaths


int readGraph(multimap<int, node>* graph, int argc, char *argv[]){
    string fileName, line;
    ifstream graphFile;
    int numNodes;
    int origin, num, weight;
    struct node n;

    std::multimap<int, node>::iterator it = graph->begin();

    if (argc == 1){
        cout << "Enter graph file name: ";
        cin >> fileName;
    }
    else if (argc >= 2){
        fileName = argv[1];
    }
    else{
        cout << "Usage: " << argv[0] << " [file name]" << endl;
        exit(EXIT_FAILURE);
    }

    graphFile.open(fileName);
    if (!graphFile.is_open()){
        cout << "Failed to open file." << endl;
        exit(EXIT_FAILURE);
    }

    getline(graphFile, line);
    numNodes = stoi(line);

    line.clear();

    while (graphFile >> origin >> num >> weight){
        n = {num, weight};
        graph->emplace(origin, n);
    }

    return numNodes;

} //end readGraph
