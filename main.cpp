#include <iostream>
#include <vector>
#include <random>
#include <iomanip>
#include <climits>

using namespace std;

class global_map
{
    int rows;
    int cols;
    int maximum_elevation;
    int minimum_elevation;

public:
    vector<vector<double>> g_map;

    global_map(int n_rows,
               int n_col,
               int max_height,
               int min_height
            ) : rows(n_rows),cols(n_col),maximum_elevation(max_height),minimum_elevation(min_height){
        vector<vector<double>> temp(rows,vector<double>(cols,0));
        g_map = std::move(temp);
        generate_map();
    }

    void generate_map()
    {   unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator (seed);
        std::uniform_real_distribution<double> distribution(minimum_elevation,maximum_elevation-1);
        for(size_t i=0; i<rows;i++)
        {
            for(size_t j=0;j<cols;j++)
            {
                g_map[i][j]  = distribution(generator);
            }
        }
        make_pit_in_map();
        //display_vector(g_map);        //See elevation map
    }

    vector<pair<int,int>> get_pit_boundary_coordinates()
    {
        //These are hard_coded values as of now to test the validation of the concept
        vector<pair<int,int>> pit_boundary{make_pair(1,6),
                                           make_pair(1,7),
                                           make_pair(2,5),
                                           make_pair(2,8),
                                           make_pair(3,6),
                                           make_pair(3,7)
        };
        return std::move(pit_boundary);
    }

    vector<pair<int,int>> get_pit_interior_coordinates()
    {
        //These are hard_coded values as of now to test the validation of the concept
        vector<pair<int,int>> pit_interior_coordinates{make_pair(2,6),
                                                       make_pair(2,7),

        };
        return std::move(pit_interior_coordinates);
    }

    vector<pair<int,int>> get_pit_bbox_coordinates()
    {
        //These are hard_coded values as of now to test the validation of the concept
        vector<pair<int,int>> pit_bbox_coordinates{make_pair(0,4),
                                                   make_pair(0,9),
                                                   make_pair(4,4),
                                                   make_pair(4,9)

        };
        return std::move(pit_bbox_coordinates);
    }

    void make_pit_in_map()
    {
        const auto pit_boundary = get_pit_boundary_coordinates();
        const auto points_in_pit = get_pit_interior_coordinates();

        for(auto x:pit_boundary)
        {
            g_map[x.first][x.second] = maximum_elevation+5;
        }

        for(auto x:points_in_pit)
        {
            g_map[x.first][x.second] = minimum_elevation-5;
        }
    }
    template <class T>
    void display_vector(const vector<vector<T>> &vec)
    {
        for(size_t i=0; i<rows;i++)
        {
            for(size_t j=0;j<cols;j++)
            {
                cout<<setprecision(2)<<vec[i][j]<<"\t";
            }
            cout<<endl;
        }
    }

    void display_final_map()
    {
        const auto pit_bbox_coordinates = get_pit_bbox_coordinates();
        const auto pit_boundary = get_pit_boundary_coordinates();
        //const auto points_in_pit = get_pit_interior_coordinates();

        vector<vector<char>> display_map(rows,vector<char>(cols,'.'));
        for(auto x:pit_boundary)
        {
            display_map[x.first][x.second] = 'X';
        }

        for(auto x:pit_bbox_coordinates)
        {
            display_map[x.first][x.second] = '#';
        }
        display_vector(display_map);
    }
};

//=====================================================================================================================

struct coordinate
{
    int x;
    int y;
    coordinate(int x_coord,
            int y_coord):
            x(x_coord),y(y_coord){}
};

//=====================================================================================================================

struct bbox
{
    int x_min;
    int y_min;
    int x_max;
    int y_max;
    bbox(int x_min_coord,
         int y_min_coord,
         int x_max_coord,
         int y_max_coord):
         x_min(x_min_coord),y_min(y_min_coord),x_max(x_max_coord),y_max(y_max_coord){}

    void get_bbox_coord(const vector<pair<int,int>> &pit_bbox)
    {
        int X_MIN = INT_MAX;
        int Y_MIN = INT_MAX;
        int X_MAX = INT_MIN;
        int Y_MAX = INT_MIN;
        for(const auto elt:pit_bbox)
        {
            auto x_coord = elt.first;
            auto y_coord = elt.second;
            if(x_coord>X_MAX)
                X_MAX = x_coord;
            else if (x_coord<X_MIN)
                X_MIN = x_coord;

            if(y_coord>Y_MAX)
                Y_MAX = y_coord;
            else if (y_coord<Y_MIN)
                Y_MIN = y_coord;
        }
        x_min = X_MIN;
        y_min = Y_MIN;
        x_max = X_MAX;
        y_max = Y_MAX;
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
    for(size_t dir = 0; dir < NUMOFDIRS; dir++) {
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
                            const vector<coordinate> &neighbors)
{
    //TO BE IMPLEMENTED
    //See if x,y is less than all its neighbors (elevation-wise) it is a pit edge.
}

//=======================================================================================================================

vector<coordinate> get_pit_edges(const vector<vector<double>> &map,
              const vector<pair<int,int>> &pit_bbox)
{
    bbox b(0,0,0,0);
    b.get_bbox_coord(pit_bbox);
//    cout<<b.x_min<<"\t"<<b.y_min<<"\t"<<b.x_max<<"\t"<<b.y_max<<endl;
    vector<coordinate> pit_edges;
    for(size_t i=b.x_min;i<=b.x_max;i++)
    {
        for(size_t j=b.y_min;j<=b.y_max;j++)
        {   //Note: Since we know that pit_bbox wont be a very large 2D vector O(n^3) is fine. See if it can be optimised
            const auto neighbors = get_neighbors(i,j,map);
                if(is_coordinate_pit_edge(i,j,map,neighbors))
                    pit_edges.push_back(coordinate(i,j));
        }
    }
    return pit_edges;
}

//======================================================================================================================

int main() {
    const int N_ROWS = 10;
    const int N_COLS = 10;
    const int MAX_ELEVATION = 100;
    const int MIN_ELEVATION= 90;
    global_map g = global_map(N_ROWS,N_COLS,MAX_ELEVATION,MIN_ELEVATION);
    g.display_final_map();
    const auto map = g.g_map;
    const auto pit_bounding_box = g.get_pit_bbox_coordinates();
    get_pit_edges(map,pit_bounding_box);
    //auto map = g.g_map;
    //auto lunar_pit = generate_random_pit();

    std::cout << "Hello, World!" << std::endl;
    return 0;
}