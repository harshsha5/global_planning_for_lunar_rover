#include <iostream>
#include <vector>
#include <iomanip>
#include <set>
#include <unordered_set>
//#include "convert_img_to_map.h"
#include "coordinate.h"
#include "b_box.h"
#include "global_map.h"
#include "planning_map.h"
#include "utility.h"

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
//    g.display_final_map();

    const coordinate start_coordinate{N_ROWS-1,0};
    const coordinate goal_coordinate{N_ROWS-2,N_COLS-2};
    planning_map my_map{g.g_map,MIN_ELEVATION,MAX_ELEVATION+10}; //Pit interiors have been made as obstacle here
    const auto path = astar(start_coordinate,goal_coordinate,my_map);

    g.path = path;
    g.display_final_map();

    return 0;
}