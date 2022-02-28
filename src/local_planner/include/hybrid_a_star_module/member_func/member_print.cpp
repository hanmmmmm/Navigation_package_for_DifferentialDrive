#include "../hybrid_astar.h"



// void Hybrid_astar_class::print_path()
// {
//     std::cout << "Find path:" << std::endl;
//     for (auto p : path)
//     {
//         std::cout << p[0] << " " << p[1] << std::endl;
//     }
// }




void Hybrid_astar_class::print_init_info()
{

    std::cout << "\nHybrid_astar start"
              << "\nfine map size:  w: " << fine_map_width_ << " h: " << fine_map_height_
              << "\ngrid map size:  w: " << grid_map_width_ << " h: " << grid_map_height_

              << "\nStart location: "
              << start_pose_[0] << " " << start_pose_[1] << " " << start_pose_[2]
              << "\nStart grid    : "
              << start_grid_[0] << " " << start_grid_[1] << " " << start_grid_[2]
              << "\nGoal location : "
              << goal_pose_[0] << " " << goal_pose_[1] << " " << goal_pose_[2]
              << "\nGoal grid     : "
              << goal_grid_[0] << " " << goal_grid_[1] << " " << goal_grid_[2]

            //   << "\n\nmotion model: \n"
            //   << motion_model_[0][0] << "  " << motion_model_[0][1] << "  " << motion_model_[0][2] << "  " << motion_model_[0][3]
            //   << "\n"
            //   << motion_model_[1][0] << "  " << motion_model_[1][1] << "  " << motion_model_[1][2] << "  " << motion_model_[1][3]
            //   << "\n"
            //   << motion_model_[2][0] << "  " << motion_model_[2][1] << "  " << motion_model_[2][2] << "  " << motion_model_[2][3]
            //   << "\n"
            //   << motion_model_[3][0] << "  " << motion_model_[3][1] << "  " << motion_model_[3][2] << "  " << motion_model_[3][3]
            //   << "\n"
            //   << motion_model_[4][0] << "  " << motion_model_[4][1] << "  " << motion_model_[4][2] << "  " << motion_model_[4][3]
            //   << "\n"
            //   << motion_model_[5][0] << "  " << motion_model_[5][1] << "  " << motion_model_[5][2] << "  " << motion_model_[5][3]

              << std::endl;
}