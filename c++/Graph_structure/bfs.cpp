#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <stack>
#include <algorithm>
// https://ocw.cs.pub.ro/courses/sd-ca/2014/laboratoare/laborator-07
// https://ocw.cs.pub.ro/courses/sd-ca/2014/laboratoare/laborator-12
// https://www.pbinfo.ro/articole/810/grafuri-neorientate
// https://lesleylai.info/en/auto-parameters/
// https://stackoverflow.com/questions/7576953/what-is-the-meaning-of-the-auto-keyword
// https://xyproblem.info/
// https://en.cppreference.com/w/cpp/language/rule_of_three
// https://en.cppreference.com/w/cpp/language/rule_of_three#Rule_of_zero
// https://stackoverflow.com/questions/29944985/is-there-a-way-to-pass-auto-as-an-argument-in-c
// https://www.doxygen.nl/manual/docblocks.html#vhdlblocks
//
//
//
using namespace std;

int var; /** Determine if the graph is bigger than 2 nodes */

/** @brief Class with functions for displaying vectors and the matrix. Also to read the matrix from the txt file*/
class visual_statements_read_matrix
{

private:
public:
    /**
     * @brief outputs the information in the graph as a matrix format
     * @param[in] matrix information of the graph in a matrix format
     */
    void print_matrix(const vector<vector<short>> &matrix)
    {
        cout << "  ";
        for (short i = 0; i < matrix.size(); i++)
            cout << " " << i;

        cout << endl
             << "  ";
        for (const auto &i : matrix[1])
            cout << "__";

        cout << endl;
        for (short i = 0; i < matrix.size(); i++)
        {
            cout << i << "| ";
            for (short j = 0; j < matrix.size(); j++)
                cout << matrix[i][j] << " ";

            cout << endl;
        }
    }

    /**
     * @brief outputs the information in the Vector
     * @param[in] Vector information that we want to display, it needs to be a list
     */
    template <typename T>
    void print_vector(const T &Vector)
    {
        for (const auto &element : Vector)
            cout << element << " ";
        cout << endl;
    }

    /**
     * @brief function for showing the content of the vector inside the Kruskal_minimum_spanning_tree function
     * @param[in] vector vector with the information that we wanna display
     */
    void print_vector_double_pair(const vector<pair<pair<short, short>, short>> &vector)
    {
        for (const auto &cost : vector)
            cout << cost.first.first << " | ";
        cout << endl;

        for (const auto &cost : vector)
            cout << cost.first.second << " | ";
        cout << endl;

        for (const auto &cost : vector)
            cout << cost.second << " | ";
        cout << endl;
    }

    /**
     * @brief  Function for reading the graph from a txt file in a matrix format
     * @param[in] filename name of the file where the square matrix of the graph is stored in the format  eq: 2 \n 1 0 \n 34 1
     * @param[in] matrix_size size of the matrix
     * @param[out] matrix where we are gonna store the information of the graph in a matrix format
     */
    void read_Matrix_From_File(const string &filename, short &matrix_size, vector<vector<short>> &matrix)
    {

        ifstream inFile(filename);

        if (inFile.is_open())
        {
            inFile >> matrix_size;

            matrix.resize(matrix_size, vector<short>(matrix_size));

            for (auto i = 0; i < matrix_size; ++i)
                for (auto j = 0; j < matrix_size; ++j)
                    inFile >> matrix[i][j];

            inFile.close();
        }
        else
            cout << "File " << filename << " couldn't be open" << endl;
    }
};

/** @brief Class with functions for checking the properties of the graph matrix. Properties: if is undirected_graph, if is a connected graph, if has at least 2 nodes*/
class graph_verification
{
private:
    vector<vector<short>> matrix;

public:
    /**
     * @brief Constructor with the conditions implemented
     * @param[out] undirected_graph True if the graph is undirected, else False
     * @param[out] connect_graph True if the graph is connected, else False
     * @param[out] at_least_2_nodes True if the graph has more than 2 nodes, else False
     * @param[in] matrix the matrix where we want to determine the conditions. Information of the graph is in a matrix format
     */
    graph_verification(bool &undirected_graph, bool &connect_graph, bool &at_least_2_nodes, const vector<vector<short>> &matrix_reference)
    {
        matrix = matrix_reference;
        is_undirected_graph(undirected_graph);
        is_a_connected_graph(connect_graph);
        is_bigger_than_2(at_least_2_nodes);
    }
    /**
     * @brief Determine if the graph is undirected or not
     * @param[out] undirected_graph True if the graph is undirected, else False
     */
    void is_undirected_graph(bool &undirected_graph)
    {

        for (short i = 0; i < matrix.size(); i++)
            for (short j = i + 1; j < matrix[i].size(); j++)
                if (matrix[i][j] != matrix[j][i])
                {
                    undirected_graph = false;
                    return;
                }
    }

    /**
     * @brief Determine if the graph is connected or not
     * @param[out] connect_graph True if the graph is connected, else False
     */
    void is_a_connected_graph(bool &connect_graph)
    {
        vector<bool> position_visited_connection;
        queue<short> list_of_nodes_visited_connected;
        short nod_start = 0;

        position_visited_connection.assign(matrix.size(), false);
        position_visited_connection[nod_start] = true;

        list_of_nodes_visited_connected.push(nod_start);

        do
        {
            short current_node = list_of_nodes_visited_connected.front();
            list_of_nodes_visited_connected.pop();
            for (auto j = 0; j < matrix[current_node].size(); j++)
                if ((matrix[current_node][j] != 0) and (position_visited_connection[j] == false) and (j != nod_start))
                {
                    position_visited_connection[j] = true;
                    list_of_nodes_visited_connected.push(j);
                }
        } while (!list_of_nodes_visited_connected.empty());

        if (position_visited_connection.size() == matrix.size())
            connect_graph = true;
    }

    /**
     * @brief Determine if the graph has more than 2 nodes
     * @param[out] at_least_2_nodes True if the graph has more than 2 nodes, else False
     */
    void is_bigger_than_2(bool &at_least_2_nodes)
    {
        if (matrix.size() > 2)
        {
            at_least_2_nodes = true;
        }
    }
};

/** @brief Class with functions for processing the graph. Function available : BFS, DFS, PRIM, Kruskal */
class Graph_algorithms
{
private:
    short matrix_size;
    vector<vector<short>> connections;
    bool undirected_graph; /** Determine if the graph is undirected */
    bool at_least_2_nodes; /** Determine if the graph is bigger than 2 nodes */
    bool connect_graph;    /** Determine if the graph is connected */

    vector<bool> position_visited_Bfs;
    vector<short> parent_node_Bfs;
    queue<short> list_of_nodes_visited_Bfs;

    vector<bool> position_visited_Dfs;

    vector<bool> position_visited_Prim_MST;
    vector<short> position_path_Prim_MST;
    vector<short> position_cost_Prim_MST;

    vector<short> position_visited_Kruskal;
    vector<pair<pair<short, short>, short>> position_cost_Kruskal;

    vector<bool> position_visited_Dijkstra;
    vector<short> position_path_Dijkstra;
    vector<short> position_cost_Dijkstra;

    visual_statements_read_matrix print;

public:
    Graph_algorithms(string txt_file_name)
    {
        print.read_Matrix_From_File(txt_file_name, matrix_size, connections);
        print.print_matrix(connections);

        graph_verification graph_properties(undirected_graph, connect_graph, at_least_2_nodes, connections);
        vector<short> bfs_list;
        // short nod_start_bfs = 2;
        // parent_node_Bfs.assign(connections.size(), -1);
        // position_visited_Bfs.assign(connections.size(), false);
        // list_of_nodes_visited_Bfs.push(nod_start_bfs);
        // position_visited_Bfs[nod_start_bfs] = true
        // bfs(nod_start_bfs, bfs_list);
        // cout << " bfs_list: ";
        // print.print_vector(bfs_list);

        // vector<short> dfs_list;
        // short nod_start_dfs = 2;
        // position_visited_Dfs.assign(connections.size(), false);
        // dfs(nod_start_dfs, dfs_list, position_visited_Dfs);
        // cout << " dfs_list: ";
        // print_vector(dfs_list);

        // trebuie facut conditie sa fie neorientat + mai mare decat doi
        // short nod_start_prim_MST = 1;
        // vector<short> Prim_minimum_spanning_tree_list;
        // Prim_minimum_spanning_tree(nod_start_prim_MST, Prim_minimum_spanning_tree_list);
        // cout << " Prim minimum spanning tree: ";
        // print.print_vector(Prim_minimum_spanning_tree_list);
        // cout << " position_visited_Prim_MST: ";
        // print.print_vector(position_visited_Prim_MST);
        // cout << " position_path_Prim_MST: ";
        // print.print_vector(position_path_Prim_MST);
        // cout << " position_cost_Prim_MST: ";
        // print.print_vector(position_cost_Prim_MST);

        short nod_start_kruskal_MST = 1;
        position_visited_Kruskal.assign(connections.size(), -1);
        short minimum_spanning_tree_cost_Kruskal = 0;

        // Kruskal_minimum_spanning_tree(nod_start_kruskal_MST, minimum_spanning_tree_cost_Kruskal);

        short nod_start_Dijkstra = 1;
        vector<vector<short>> minimum_cost_path_Dijkstra;
        Dijkstra_minimum_cost_path(nod_start_Dijkstra, minimum_cost_path_Dijkstra);
    }

    /**
     * @brief Breadth First Search algorithm implemented. O(N)
     * @param[in] nod_start the node where the search will begin from
     * @param[out] bfs_list list with the result of the search
     */
    void bfs(const short &nod_start, vector<short> &bfs_list)
    {
        if (list_of_nodes_visited_Bfs.empty())
            return;

        short current_node = list_of_nodes_visited_Bfs.front();
        bfs_list.push_back(current_node);
        list_of_nodes_visited_Bfs.pop();
        for (auto j = 0; j < connections[current_node].size(); j++)
            if ((connections[current_node][j] != 0) and (position_visited_Bfs[j] == false) and (j != nod_start))
            {
                position_visited_Bfs[j] = true;
                parent_node_Bfs[j] = current_node;
                list_of_nodes_visited_Bfs.push(j);
            }

        bfs(nod_start, bfs_list);
    }

    /**
     * @brief Depth First Search algorithm implemented. O(1^N)
     * @param[in] nod_start the node where the search will begin from
     * @param[out] position_visited_Dfs_reference list with the result of the search
     * @see https://www.pbinfo.ro/articole/5511/parcurgerea-in-adancime
     * @see https://web.stanford.edu/class/archive/cs/cs106b/cs106b.1176/handouts/midterm/5-BigO.pdf
     */
    void dfs(const short &nod_start, vector<short> &dfs_list, vector<bool> &position_visited_Dfs_reference)
    {
        position_visited_Dfs_reference[nod_start] = true;
        dfs_list.push_back(nod_start);
        for (auto i = 0; i <= connections.size(); i++)
            if ((connections[nod_start][i] != 0) and (position_visited_Dfs_reference[i] == false))
                dfs(i, dfs_list, position_visited_Dfs_reference);
    }

    /**
     * @brief Determine the minimum spanning tree using Prim algorithm. O(N^2) it can be obtain O(NlogN)
     * @param[in] nod_start the node where the search will begin from
     * @param[out] Prim_minimum_spanning_tree_list list with the result of the search
     * @see https://www.cs.usfca.edu/~galles/visualization/Prim.html
     * @see https://www.pbinfo.ro/articole/2443/algoritmul-lui-prim
     * @see https://www.geeksforgeeks.org/prims-mst-for-adjacency-list-representation-greedy-algo-6/
     */
    void Prim_minimum_spanning_tree(const short &nod_start, vector<short> &Prim_minimum_spanning_tree_list)
    {
        position_visited_Prim_MST.assign(connections.size(), false);
        position_path_Prim_MST.assign(connections.size(), -1);
        position_cost_Prim_MST.assign(connections.size(), SHRT_MAX);

        position_cost_Prim_MST[nod_start] = 0;
        position_path_Prim_MST[nod_start] = -1;
        position_visited_Prim_MST[nod_start] = true;
        Prim_minimum_spanning_tree_list.push_back(nod_start);

        short current_position = nod_start;
        short lowest_cost = SHRT_MAX;
        short lowest_cost_position = 0;

        do
        {

            for (short j = 0; j < connections[current_position].size(); j++)
                if ((0 != connections[current_position][j]) and (position_visited_Prim_MST[j] == false))
                {
                    position_path_Prim_MST[j] = current_position;
                    position_cost_Prim_MST[j] = connections[current_position][j];
                }

            lowest_cost = SHRT_MAX;

            for (short j = 0; j < position_cost_Prim_MST.size(); j++)
                if ((position_cost_Prim_MST[j] < lowest_cost) and (position_visited_Prim_MST[j] == false))
                {
                    lowest_cost = position_cost_Prim_MST[j];
                    lowest_cost_position = j;
                }

            // if there is any node that is not connected to the the graph we need to find it out
            auto result = find(Prim_minimum_spanning_tree_list.begin(), Prim_minimum_spanning_tree_list.end(), lowest_cost_position);

            if (result == Prim_minimum_spanning_tree_list.end())
            {
                position_visited_Prim_MST[lowest_cost_position] = true;
                Prim_minimum_spanning_tree_list.push_back(lowest_cost_position);
                current_position = lowest_cost_position;
            }
            else // if node is not connected with the graph connected to the nod_start we need to abandone the execution
                break;

        } while (Prim_minimum_spanning_tree_list.size() != connections.size());
    }

    /**
     * @brief Determine the minimum spanning tree using Kruskal algorithm.
     * @param[in] nod_start the node where the search will begin from
     * @param[out] minimum_spanning_tree_cost_Kruskal_reference list with the result of the search
     * @see https://www.pbinfo.ro/articole/19048/algoritmul-lui-kruskal
     * @see https://arborecuradacina.weebly.com/arborele-de-cost-minim--algoritmul-kruskal.html
     */
    void Kruskal_minimum_spanning_tree(const short &nod_start, short &minimum_spanning_tree_cost_Kruskal_reference)
    {

        for (short i = 0; i < connections.size(); i++)
            for (short j = i + 1; j < connections[i].size(); j++)
                if ((connections[i][j] != 0) and (i != j))
                    position_cost_Kruskal.push_back({{i, j}, connections[i][j]});

        sort(position_cost_Kruskal.begin(), position_cost_Kruskal.end(),
             [](const pair<pair<short, short>, short> &a, const pair<pair<short, short>, short> &b)
             {
                 return a.second < b.second;
             });

        cout << "kruskal list ordering: " << endl;
        print.print_vector_double_pair(position_cost_Kruskal);

        for (const auto &cost : position_cost_Kruskal)
        {
            if ((position_visited_Kruskal[cost.first.first] == -1) or (position_visited_Kruskal[cost.first.second] == -1))
            {
                if (position_visited_Kruskal[cost.first.second] != -1)
                    position_visited_Kruskal[cost.first.first] = position_visited_Kruskal[cost.first.second];
                else if (position_visited_Kruskal[cost.first.first] != -1)
                    position_visited_Kruskal[cost.first.second] = position_visited_Kruskal[cost.first.first];
                else
                {
                    position_visited_Kruskal[cost.first.first] = cost.first.first;
                    position_visited_Kruskal[cost.first.second] = cost.first.first;
                }
                minimum_spanning_tree_cost_Kruskal_reference = minimum_spanning_tree_cost_Kruskal_reference + cost.second;
            }
            else
            {

                if (position_visited_Kruskal[cost.first.first] != position_visited_Kruskal[cost.first.second])
                {
                    minimum_spanning_tree_cost_Kruskal_reference = minimum_spanning_tree_cost_Kruskal_reference + cost.second;
                    short copy_value_on_position = position_visited_Kruskal[cost.first.first];
                    for (short i = 0; i < position_visited_Kruskal.size(); i++)
                        if (position_visited_Kruskal[i] == copy_value_on_position)
                            position_visited_Kruskal[i] = position_visited_Kruskal[cost.first.second];
                }
            }
        }

        cout << "Kruskal visited positions: ";
        print.print_vector(position_visited_Kruskal);
    }

    /**
     * @brief Determine the minimum cost from nod_start to every node of the graph. We are using the Dijkstra algorithm
     * @param[in] nod_start the node where the search will begin from
     * @param[out] minimum_cost_path_Dijkstra matrix with the cost from nod_start to every node,
     * type nod_start path_of_nodes size_of_paths; eq:  0 1 4 7 3. Nod_start = 0 , Nod_finish = 7, path = 0-1-4-7 size_of_paths = 3
     * @see https://www.pbinfo.ro/articole/6135/algoritmul-lui-dijkstra
     * @see https://www.cs.usfca.edu/~galles/visualization/Dijkstra.html
     */
    void Dijkstra_minimum_cost_path(const short &nod_start, vector<vector<short>> minimum_cost_path_Dijkstra)
    {
        position_visited_Dijkstra.assign(connections.size(), false);
        position_path_Dijkstra.assign(connections.size(), -1);
        position_cost_Dijkstra.assign(connections.size(), SHRT_MAX);
        short finished = false;
        short current_position = nod_start;
        short lowest_cost = SHRT_MAX;
        short lowest_cost_position = 0;
        position_cost_Dijkstra[nod_start] = 0;
        position_path_Dijkstra[nod_start] = -1;
        position_visited_Dijkstra[nod_start] = true;

        cout << "Dijkstra" << endl;
        do
        {
            for (short j = 0; j < connections[current_position].size(); j++)
            {
                if ((0 != connections[current_position][j]) and (position_visited_Dijkstra[j] == false))
                {
                    position_path_Dijkstra[j] = current_position;
                    position_cost_Dijkstra[j] = connections[current_position][j];
                    cout << "; position_cost_Dijkstra[j]: " << position_cost_Dijkstra[j] << ", position_path_Dijkstra[j]: " << position_path_Dijkstra[j];
                }
            }

            lowest_cost = SHRT_MAX;

            for (short j = 0; j < position_cost_Dijkstra.size(); j++)
            {
                if ((position_cost_Dijkstra[j] < lowest_cost) and (position_visited_Dijkstra[j] == false))
                {
                    lowest_cost = position_cost_Dijkstra[j];
                    lowest_cost_position = j;
                }
            }
            // if there is any node that is not connected to the the graph we need to find it out
            cout << "position_visited_Dijkstra: ";
            print.print_vector(position_visited_Dijkstra);
            cout << "position_cost_Dijkstra: ";
            print.print_vector(position_cost_Dijkstra);
            cout << "position_path_Dijkstra: ";
            print.print_vector(position_path_Dijkstra);

            if (lowest_cost != SHRT_MAX)
            {
                position_visited_Dijkstra[lowest_cost_position] = true;
                current_position = lowest_cost_position;
            }
            else // if node is not connected with the graph connected to the nod_start we need to abandone the execution
                finished = true;

            cout << endl;

        } while (!finished);
    }

    /**
     * @brief Determine the minimum cost from nod_start to every node of the graph. We are using the Dijkstra algorithm
     * @param[in] nod_start the node where the search will begin from
     * @param[out] minimum_cost_path_Dijkstra matrix with the cost from nod_start to every node,
     * type nod_start path_of_nodes size_of_paths; eq:  0 1 4 7 3. Nod_start = 0 , Nod_finish = 7, path = 0-1-4-7 size_of_paths = 3
     * @see https://www.pbinfo.ro/articole/5891/drumuri-de-cost-minim-intr-un-graf-orientat
     * @see https://www.pbinfo.ro/articole/5887/algoritmul-roy-warshall-floyd
     * @see https://www.cs.usfca.edu/~galles/visualization/Floyd.html
     */
    void warshall_floyd(const short &b, vector<vector<short>> &bb)
    {
    }
};

int main()
{
    string txt_file = "matrix_dijkstra copy.txt";
    Graph_algorithms g(txt_file);
}