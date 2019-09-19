#include <iostream>
#include <vector>
#include <random>
#include <iomanip>

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
        display_elevation_map();
    }

    void make_pit_in_map()
    {
        //Hard coding the pit boundary as of now to test code
        const vector<pair<int,int>> pit_boundary{make_pair(1,6),
                               make_pair(1,7),
                               make_pair(2,5),
                               make_pair(2,8),
                               make_pair(3,6),
                               make_pair(3,7)
                               };

        const vector<pair<int,int>> points_in_pit{make_pair(2,6),
                                           make_pair(2,7),
        };

        for(auto x:pit_boundary)
        {
            g_map[x.first][x.second] = maximum_elevation+5;
        }

        for(auto x:points_in_pit)
        {
            g_map[x.first][x.second] = minimum_elevation-5;
        }
    }

    void display_elevation_map()
    {
        for(size_t i=0; i<rows;i++)
        {
            for(size_t j=0;j<cols;j++)
            {
                //g_map[i][j] = (rand() % (maximum_elevation + 1 - minimum_elevation)) + minimum_elevation;
                cout<<setprecision(2)<<g_map[i][j]<<"\t";
            }
            cout<<endl;
        }
    }

    vector<pair<int,int>> get_pit_bbox_coordinates()
    {
        //These are hard_coded values as of now to test the validation of the concept
        
    }
};

//======================================================================================================================

int main() {
    const int N_ROWS = 10;
    const int N_COLS = 10;
    const int MAX_ELEVATION = 100;
    const int MIN_ELEVATION= 90;
    global_map g = global_map(N_ROWS,N_COLS,MAX_ELEVATION,MIN_ELEVATION);

    //auto map = g.g_map;
    //auto lunar_pit = generate_random_pit();

    std::cout << "Hello, World!" << std::endl;
    return 0;
}