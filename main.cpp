#include <iostream>
#include <vector>
#include <iomanip>
#include <climits>
#include <set>
#include <unordered_set>
//#include "convert_img_to_map.h"
#include "coordinate.h"
#include "b_box.h"
#include "global_map.h"

using namespace std;

//=====================================================================================================================
/// Global variables

int MAP_LENGTH;
int MAP_WIDTH;

//=====================================================================================================================

struct coordinate_hasher
{
    size_t
    operator()(const coordinate &obj) const
    {
        return std::hash<int>()(obj.x * MAP_WIDTH + obj.y);
    }
};

//=====================================================================================================================

//class global_map
//{
//    int rows;
//    int cols;
//    int maximum_elevation;
//    int minimum_elevation;
//
//public:
//    vector<vector<double>> g_map;
//    vector<coordinate> way_points;
//
//    global_map(int n_rows,
//               int n_col,
//               int max_height,
//               int min_height
//            ) : rows(n_rows),cols(n_col),maximum_elevation(max_height),minimum_elevation(min_height){
//        vector<vector<double>> temp(rows,vector<double>(cols,0));
//        g_map = std::move(temp);
//        generate_map();
//    }
//
//    void generate_map()
//    {   unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
//        std::default_random_engine generator (seed);
//        std::uniform_real_distribution<double> distribution(minimum_elevation,maximum_elevation-1);
//        for(size_t i=0; i<rows;i++)
//        {
//            for(size_t j=0;j<cols;j++)
//            {
//                g_map[i][j]  = distribution(generator);
//            }
//        }
//        make_pit_in_map();
//        //display_vector(g_map);        //See elevation map
//    }
//
//    vector<pair<int,int>> get_pit_boundary_coordinates()
//    {
//        //These are hard_coded values as of now to test the validation of the concept
//        vector<pair<int,int>> pit_boundary{make_pair(2,9),
//                                           make_pair(2,10),
//                                           make_pair(2,11),
//                                           make_pair(2,12),
//                                           make_pair(2,13),
//                                           make_pair(2,14),
//                                           make_pair(6,9),
//                                           make_pair(6,10),
//                                           make_pair(6,11),
//                                           make_pair(6,12),
//                                           make_pair(3,8),
//                                           make_pair(3,14),
//                                           make_pair(4,7),
//                                           make_pair(4,14),
//                                           make_pair(5,8),
//                                           make_pair(5,13),
//                                           make_pair(2,14),
//        };
//        return std::move(pit_boundary);
//    }
//
//    vector<coordinate> get_pit_interior_coordinates()
//    {
//        //These are hard_coded values as of now to test the validation of the concept
//        const vector<pair<int,int>> pit_interior_coordinates{make_pair(3,9),
//                                                             make_pair(3,10),
//                                                             make_pair(3,11),
//                                                             make_pair(3,12),
//                                                             make_pair(3,13),
//                                                             make_pair(4,8),
//                                                             make_pair(4,9),
//                                                             make_pair(4,10),
//                                                             make_pair(4,11),
//                                                             make_pair(4,12),
//                                                             make_pair(4,13),
//                                                             make_pair(5,9),
//                                                             make_pair(5,10),
//                                                             make_pair(5,11),
//                                                             make_pair(5,12)
//
//        };
//
//        vector<coordinate> pit_interior_coords;
//        for(const auto &coord: pit_interior_coordinates)
//        {
//            pit_interior_coords.emplace_back(coordinate(coord.first,coord.second));
//        }
//        return std::move(pit_interior_coords);
//    }
//
//    vector<pair<int,int>> get_pit_bbox_coordinates()
//    {
//        //These are hard_coded values as of now to test the validation of the concept
//        vector<pair<int,int>> pit_bbox_coordinates{make_pair(1,5),
//                                                   make_pair(1,16),
//                                                   make_pair(8,5),
//                                                   make_pair(8,16)
//
//        };
//        return std::move(pit_bbox_coordinates);
//    }
//
//    void make_pit_in_map()
//    {
//        const auto pit_boundary = get_pit_boundary_coordinates();
//        const auto points_in_pit = get_pit_interior_coordinates();
//
//        for(auto x:pit_boundary)
//        {
//            g_map[x.first][x.second] = maximum_elevation+5;
//        }
//
//        for(auto point_in_pit:points_in_pit)
//        {
//            g_map[point_in_pit.x][point_in_pit.y] = minimum_elevation-5;
//        }
//    }
//    template <class T>
//    void display_vector(const vector<vector<T>> &vec)
//    {
//        for(int i=0; i<rows;i++)
//        {
//            for(int j=0;j<cols;j++)
//            {
//                cout<<setprecision(2)<<vec[i][j]<<"\t";
//            }
//            cout<<endl;
//        }
//    }
//
//    void display_final_map()
//    {
//        const auto pit_bbox_coordinates = get_pit_bbox_coordinates();
//        const auto pit_boundary = get_pit_boundary_coordinates();
//        //const auto points_in_pit = get_pit_interior_coordinates();
//
//        vector<vector<char>> display_map(rows,vector<char>(cols,'.'));
//        for(auto x:pit_boundary)
//        {
//            display_map[x.first][x.second] = 'X';
//        }
//
//        for(auto x:pit_bbox_coordinates)
//        {
//            display_map[x.first][x.second] = '#';
//        }
//
//        if(!way_points.empty())
//        {
//            for(const auto &waypoint:way_points)
//            {
//                display_map[waypoint.x][waypoint.y] = '?';
//            }
//        }
//        display_vector(display_map);
//    }
//
//    int get_maximum_elevation()
//    {
//        return maximum_elevation;
//    }
//
//    int get_minimum_elevation()
//    {
//        return minimum_elevation;
//    }
//};

//======================================================================================================================

vector<coordinate> get_neighbors(const int &x,
                const int &y,
                const vector<vector<double>> &map)
{
    constexpr int NUMOFDIRS = 4; //Assume 4 connected grid for now
    int dX[NUMOFDIRS] = {-1, 1, 0, 0};
    int dY[NUMOFDIRS] = {0, 0, -1, 1};
    vector<coordinate> neighbors;
    for(int dir = 0; dir < NUMOFDIRS; dir++) {
        int newx = x + dX[dir];
        int newy = y + dY[dir];
        if (newx >= 0 && newx < map.size() && newy >= 0 && newy < map[0].size())
        {
            neighbors.emplace_back(coordinate(newx,newy));
        }
    }
    return std::move(neighbors);
}

//======================================================================================================================

bool is_coordinate_pit_edge(const int &x,
                            const int &y,
                            const vector<vector<double>> &map,
                            const vector<coordinate> &neighbors,
                            const int &threshold)
{
    for(const auto &neighbor:neighbors)
    {
        if(map[x][y] - map[neighbor.x][neighbor.y]>threshold)
            return true;
    }
    return false;
    //TO BE IMPLEMENTED
    //See if x,y is less than all its neighbors (elevation-wise) it is a pit edge.
}

//=======================================================================================================================

void dfs_util(unordered_set<coordinate,coordinate_hasher> &accept_list,
              unordered_set<coordinate,coordinate_hasher> &reject_list,
              unordered_set<coordinate,coordinate_hasher> &visited,
              const int &depth,
              const vector<vector<double>> &map,
              const coordinate &present_coordinate,
              const int &threshold)
{
    visited.insert(present_coordinate);
    const auto neighbors = get_neighbors(present_coordinate.x,present_coordinate.y,map);
    for(const auto &neighbor:neighbors)
    {
        //cout<<"=================================================="<<endl;
        //neighbor.print_coordinate();

        if(reject_list.count(neighbor)!=0)
        {
//            cout<<"Vertex already in reject list"<<endl;
            continue;
        }

        if(depth+1==threshold)
        {
            if(accept_list.count(neighbor)==0)
                {
                    accept_list.insert(neighbor);
//                    cout<<"Adding vertex to accept_list"<<endl;
                }
            continue;
        }

        if(visited.count(neighbor)==0)
        {
            reject_list.insert(neighbor);
//            cout<<"Adding to reject list"<<endl;
            if(accept_list.count(neighbor)!=0)
            {
                accept_list.erase(neighbor);
//                cout<<"Vertex removed from accept list"<<endl;
            }
            if(depth+1<threshold)
                dfs_util(accept_list,reject_list,visited,depth+1,map,neighbor,threshold);
        }
        else
        {
            cout<<"Already Visited"<<endl;
        }
    }
}

//=======================================================================================================================

vector<coordinate> generate_way_points(const vector<coordinate> &pit_edges,
                                        const vector<vector<double>> &map,
                                        const int &threshold,
                                        const vector<coordinate> &pit_interior)
{
    unordered_set<coordinate,coordinate_hasher> accept_list;
    unordered_set<coordinate,coordinate_hasher> reject_list;
    for(const auto &pit_edge:pit_edges)
    {
        reject_list.insert(pit_edge);
        //pit_edge.print_coordinate();
    }

    for(const auto &coordinate_in_pit:pit_interior)
    {
        reject_list.insert(coordinate_in_pit);
        //coordinate_in_pit.print_coordinate();
    }

    for(auto const &pit_edge: pit_edges)
    {
        unordered_set<coordinate,coordinate_hasher> visited;
        int depth = 0;
        dfs_util(accept_list,reject_list,visited,depth,map,pit_edge,threshold);
    }
    cout<<"================================"<<endl;
    vector<coordinate> way_points;
    way_points.reserve(accept_list.size());
    //cout<<"Size of way points: "<<accept_list.size()<<endl;
    for(const auto &elt:accept_list)
    {
        way_points.push_back(elt);
        elt.print_coordinate();
    }

    return std::move(way_points);
}

//=======================================================================================================================

vector<coordinate> get_pit_edges(const vector<vector<double>> &map,
              const vector<pair<int,int>> &pit_bbox,
              const int &threshold)
{
//  The threshold as of now is based on the difference of the max and min elevation

    bbox b(0,0,0,0);
    b.get_bbox_coord(pit_bbox);
//    cout<<b.x_min<<"\t"<<b.y_min<<"\t"<<b.x_max<<"\t"<<b.y_max<<endl;
    vector<coordinate> pit_edges;
    for(size_t i=b.x_min;i<=b.x_max;i++)
    {
        for(size_t j=b.y_min;j<=b.y_max;j++)
        {   //Note: Since we know that pit_bbox wont be a very large 2D vector O(n^3) is fine. See if it can be optimised
            const auto neighbors = get_neighbors(i,j,map);
            if(is_coordinate_pit_edge(i,j,map,neighbors,threshold))
                pit_edges.emplace_back(coordinate(i,j));
        }
    }
    return pit_edges;
}

//======================================================================================================================

vector<coordinate> get_feasible_waypoints(const unordered_set<coordinate,coordinate_hasher> &this_quarter_waypoints,
                                          const vector<vector<double>> &map,
                                          const double mean_elevation,
                                          const double &standard_deviation_threshold)
{
    vector<coordinate> result;
    double  standardDeviation=0;
    for(const auto &elt:this_quarter_waypoints)
        standardDeviation += pow(map[elt.x][elt.y] - mean_elevation, 2);
    standardDeviation = pow(standardDeviation, 0.5);
    cout<<"Higher bound: "<<mean_elevation+(standard_deviation_threshold*standardDeviation)<<endl;
    cout<<"Lower bound: "<<mean_elevation-(standard_deviation_threshold*standardDeviation)<<endl;

    for(const auto &elt:this_quarter_waypoints)
    {
//        cout<<"Elevation at this point is: "<<map[elt.x][elt.y]<<endl;
        if(map[elt.x][elt.y]<mean_elevation+(standard_deviation_threshold*standardDeviation) &&
                map[elt.x][elt.y]>mean_elevation-(standard_deviation_threshold*standardDeviation))
        {
            result.push_back(elt);
//            cout<<"Acccepted"<<endl;
        }
    }
    return std::move(result);
}
//======================================================================================================================

vector<vector<coordinate>> get_quarter_waypoints(const vector<coordinate> &possible_waypoints,
                      const vector<pair<int,int>> &pit_bbox,
                      const vector<vector<double>> &map,
                      const int map_width,
                      const double standard_deviation_threshold = 1
                      )
{
    bbox b(0,0,0,0);
    b.get_bbox_coord(pit_bbox);
    const auto mid_x = b.get_mid_x();
    const auto mid_y = b.get_mid_y();
    unordered_set<coordinate,coordinate_hasher> possible_waypoints_set;
    for(auto x:possible_waypoints)
    {
        possible_waypoints_set.insert(x);
    }
    vector<vector<coordinate>> result;
    unordered_set<coordinate,coordinate_hasher> this_quarter_waypoints;
    double elevation_sum = 0;
    int num_elt = 0;
    //Quarter 1 Waypoints
    for(auto it = possible_waypoints_set.begin(); it != possible_waypoints_set.end();)
    {
        if(it->x<=mid_x && it->y<=mid_y)
        {
            this_quarter_waypoints.insert(*it);
            elevation_sum+=map[it->x][it->y];
            possible_waypoints_set.erase(it++);
            num_elt++;
        }
        else
            it++;
    }
    result.emplace_back(get_feasible_waypoints(this_quarter_waypoints,map,elevation_sum/num_elt,standard_deviation_threshold));

    //Quarter 2 Waypoints
    this_quarter_waypoints.clear();
    elevation_sum = 0;
    num_elt = 0;
    for(auto it = possible_waypoints_set.begin(); it != possible_waypoints_set.end();)
    {
        if(it->x<=mid_x && it->y>mid_y)
        {
            this_quarter_waypoints.insert(*it);
            elevation_sum+=map[it->x][it->y];
            possible_waypoints_set.erase(it++);
            num_elt++;
        }
        else
            it++;
    }
    result.emplace_back(get_feasible_waypoints(this_quarter_waypoints,map,elevation_sum/num_elt,standard_deviation_threshold));

    //Quarter 3 Waypoints
    this_quarter_waypoints.clear();
    elevation_sum = 0;
    num_elt = 0;
    for(auto it = possible_waypoints_set.begin(); it != possible_waypoints_set.end();)
    {
        if(it->x>mid_x && it->y>=mid_y)
        {
            this_quarter_waypoints.insert(*it);
            elevation_sum+=map[it->x][it->y];
            possible_waypoints_set.erase(it++);
            num_elt++;
        }
        else
            it++;
    }
    result.emplace_back(get_feasible_waypoints(this_quarter_waypoints,map,elevation_sum/num_elt,standard_deviation_threshold));

    //Quarter 4 Waypoints
    this_quarter_waypoints.clear();
    elevation_sum = 0;
    num_elt = 0;
    for(auto it = possible_waypoints_set.begin(); it != possible_waypoints_set.end();)
    {
        if(it->x>mid_x && it->y<mid_y)
        {
            this_quarter_waypoints.insert(*it);
            elevation_sum+=map[it->x][it->y];
            possible_waypoints_set.erase(it++);
            num_elt++;
        }
        else
            it++;
    }
    result.emplace_back(get_feasible_waypoints(this_quarter_waypoints,map,elevation_sum/num_elt,standard_deviation_threshold));
    return std::move(result);
}

//======================================================================================================================

int main() {
    const int N_ROWS = 20;
    const int N_COLS = 20;
    const int MAX_ELEVATION = 100;
    const int MIN_ELEVATION= 90;
    MAP_LENGTH = N_ROWS;
    MAP_WIDTH = N_COLS;
    global_map g = global_map(N_ROWS,N_COLS,MAX_ELEVATION,MIN_ELEVATION);
    g.display_final_map();
    const auto map = g.g_map;
    const auto pit_bounding_box = g.get_pit_bbox_coordinates();
    const auto threshold = g.get_maximum_elevation()-g.get_minimum_elevation();

    // TILL HERE IS THE DATA THAT I WILL HAVE ALREADY- THIS INCLUDES THE MAP AND THE PIT BOUNDING BOX
    const auto pit_edges = get_pit_edges(map,pit_bounding_box,threshold);
    /// Pit interior needs to be implemented by you! You won't be given this. Just using ground truth as of now
    const auto pit_interior_points = g.get_pit_interior_coordinates();

//    for(auto pit_edge:pit_edges) //Validate pit edge
//    {
//        cout<<pit_edge.x<<"\t"<<pit_edge.y<<endl;
//    }

    // Use the pit edges. Run a DFS from each of the vertices | Validate if that state is within the map and also is not
    // a pit edge or in the interior of pit. If it's distance is less than 'x' keep it in the reject_set, if it's 'x' add it to
    // to the accept set. Also keep checking if the new vertex being encountered has a distance less than x and that state is in
    // the accept_set. Remove it from there.
    // Add the pit edge and the pit interior points in the reject list

    const int threshold_dist_from_pit{1};
    const double standard_deviation_threshold{.5};
    auto way_points = generate_way_points(pit_edges,map,threshold_dist_from_pit,pit_interior_points);

    for(const auto &p: way_points)
    {
        g.way_points.emplace_back(p);
    }
    /// There are known fallacies with depth>=2. This is because we need to elimninate those waypoints as well which maybe
    /// 2 or more depth away from point A but then come close to some other point. A check has been put in place for this,
    /// but the issue is that if the closer vertex is already expanded, then that check fails. Work on this if need be
    /// because as of now we deal with depth==1 only
    g.display_final_map();
    g.way_points.clear();
    auto quarter_waypoints = get_quarter_waypoints(way_points,pit_bounding_box,map,standard_deviation_threshold);
    for(const auto &quarter_vec: quarter_waypoints)
    {
        for(const auto &elt:quarter_vec)
            g.way_points.emplace_back(elt);
    }
    g.display_final_map();
    return 0;
}