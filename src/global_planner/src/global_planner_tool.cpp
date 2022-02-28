


#include "../include/global_planner/global_planner_node.h"




void globalHandle::inflate( vector<int8_t>& grids, const int x, const int y, const int map_width, const int map_height, const vector<vector<int8_t>>& inflate_sample_in , const int radius  )
{
    // int x_start = x - radius;
    // int x_end   = x + radius + 1 ;
    // int y_start = y - radius;
    // int y_end   = y + radius + 1 ;
    // int width = radius*2+1;

    int x_start, x_end, y_start, y_end;
    int x_start_sam, x_end_sam, y_start_sam, y_end_sam;

    if( x-radius < 0)
    {
        x_start = 0;
        x_start_sam = radius - x;
    }
    else
    {
        x_start = x-radius;
        x_start_sam = 0;
    }

    if ( x+radius > map_width )
    {
        x_end = map_width;
        x_end_sam = 2*radius - ( radius- ( map_width - 1 - x ) );
    }
    else
    {
        x_end = x+radius;
        x_end_sam = 2*radius;
    }


    if( y-radius < 0)
    {
        y_start = 0;
        y_start_sam = radius - y;
    }
    else
    {
        y_start = y-radius;
        y_start_sam = 0;
    }

    if ( y+radius > map_height )
    {
        y_end = map_height;
        y_end_sam = 2*radius - ( radius- ( map_height - 1 - y ) );
    }
    else
    {
        y_end = y+radius;
        y_end_sam = 2*radius;
    }


    // int x_start = std::max( x - radius, 0 );
    // int x_end   = std::min( x + radius, map_info_.grid_width );
    // int y_start = std::max( y - radius, 0 );
    // int y_end   = std::min( y + radius, map_info_.grid_height );
    int width  = x_end - x_start +1 ;
    int height = y_end - y_start +1 ;

    // cout << "inflate: " << x << " : " << y << "  " << x_start << " : " << x_end  << "  " << y_start << " : " << y_end << endl;
    // cout << "sample : " <<  x_start_sam << " : " << x_end_sam  << "  " << y_start_sam << " : " << y_end_sam << endl;

    vector<int> x_m, y_m, x_s, y_s;

    int xth = x_start ;
    int yth = y_start ;

    // cout << "map x " << endl;
    while (xth <= x_end)
    {
        // cout << xth << endl;
        x_m.push_back(xth);
        xth++;
    }
    // cout << "map y " << endl;
    while (yth <= y_end)
    {
        // cout << yth << endl;
        y_m.push_back(yth);
        yth++;
    }

    int xth_sam = x_start_sam ;
    int yth_sam = y_start_sam ;
    // cout << "sap x " << endl;
    while (xth_sam <= x_end_sam)
    {
        // cout << xth_sam << endl;
        x_s.push_back(xth_sam);
        xth_sam++;
    }
    // cout << "sap y " << endl;
    while (yth_sam <= y_end_sam)
    {
        // cout << yth_sam << endl;
        y_s.push_back(yth_sam);
        yth_sam++;
    }


    for(int y_fill = 0; y_fill<height; y_fill ++)
    {
        // cout << "y_fill " << y_fill << endl;
        int yys = y_s[y_fill];
        int yym = y_m[y_fill];
        vector<int8_t> arow = inflate_sample_in[ yys ];
        for( int x_fill = 0; x_fill<width; x_fill ++ )
        {
            // cout << "x_fill " << x_fill << endl;
            int xxs = x_s[x_fill];
            int xxm = x_m[x_fill];
            if( arow[ xxs ] !=0)
            {
                grids[ yym*map_width + xxm ] = arow[xxs];
            }
        }
    }
    

    // while (yth <= y_end)
    // {
    //     vector<int8_t> arow = inflate_sample_in[ yth-y_start ];
    //     int start_index = yth*map_info_.grid_width +  x_start;
    //     int xcounter =0;
    //     while (xth <= x_end)
    //     {
    //         if( arow[ xcounter ] != 0)
    //         {
    //             grids[ start_index + xcounter ] = arow[xcounter];
    //         }
    //         xcounter ++;
    //     }
        

    // }

    // int ycounter = 0;
    // for(auto arow : inflate_sample_in)
    // {
    //     int yth = y_start+ycounter;
    //     int start_index = yth*map_info_.grid_width +  x_start;
    //     int xcounter =0;
    //     while (xcounter < width)
    //     {
    //         if(arow[xcounter] != 0)
    //         {
    //             grids[ start_index + xcounter ] = arow[xcounter];
                
    //         }
    //         xcounter ++;
    //     }
    //     ycounter ++ ;
    // }
    
}



void globalHandle::build_inflate_sample( std::vector< std::vector<int8_t> >& inflate_sample_target, const int radius )
{
    int step_size = 100/(radius+1);
    int center = radius;

    for(int i = 0; i<radius*2+1; i++)
    {
        std::vector<int8_t> a_row;
        for(int j = 0; j<radius*2+1; j++)
        {
            float dx = i-center;
            float dy = j-center;
            if( dx*dx + dy*dy < radius*radius )
            {
                a_row.push_back(100);
                cout << "1 ";
            }
            else
            {
                a_row.push_back(0);
                cout << "0 ";
            }
        }
        cout << endl;
        inflate_sample_target.push_back(a_row);
    }
}


float globalHandle::rectify( float a)
{
    float angle = a;
    while (angle > 2*M_PI)
    {
        angle -= 2*M_PI;
    }
    while (angle < 0)
    {
        angle += 2*M_PI;
    }
    return angle;
}














// void globalHandle::get_tf_laser_to_base(std::array<float, 3> &the_tf)
// {
//     tf::TransformListener listener;
//     tf::StampedTransform transform;
//     bool got_tf = false;

//     while (!got_tf)
//     {
//         try
//         {
//             listener.lookupTransform("/base_link", "/base_laser_link", ros::Time(0), transform);
//             got_tf = true;
//         }
//         catch (tf::TransformException &ex)
//         {
//             ROS_ERROR("%s", ex.what());
//             ros::Duration(1.0).sleep();
//             continue;
//         }
//     }

//     the_tf[0] = transform.getOrigin().x();
//     the_tf[1] = transform.getOrigin().y();
//     the_tf[2] = transform.getRotation().x();

//     // cout << the_tf[0] << " "  << the_tf[1] << " " << the_tf[2] << endl;
// }






