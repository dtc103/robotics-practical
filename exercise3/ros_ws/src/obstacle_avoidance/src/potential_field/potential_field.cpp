#include <potential_field/potential_field.h>

PotentialField::PotentialField(Vec2f goal_position, double k_att, double k_rep, double rho_0, size_t segments):goal_position(goal_position), k_att(k_att), k_rep(k_rep), rho_0(rho_0), segments(segments){
    
}
Vec2f PotentialField::get_f_att(Vec2f current_position){
    return -this->k_att * (current_position - this->goal_position);
}

std::vector<Vec2f> PotentialField::get_f_rep(Vec2f current_pos, double curr_yaw, std::vector<Vec2f> laser_positions, double rho_0){
    auto compare_func_idx_based = [&](const int& idx){
        auto x = laser_positions[idx].x;
        auto y = laser_positions[idx].y;

        double magic_number = 100000.0;
        if(x > magic_number || x < -magic_number){
            x = magic_number;
        }
        if(y > magic_number || y < -magic_number){
            y = magic_number;
        }

        auto norm = (Vec2f(x, y) - current_pos).norm();
        if(norm > magic_number){
            return magic_number;
        }
        return norm;
    };

    // save index of lasaer_positioins and corresponding angles and norm
    std::vector<std::tuple<int, double, double>> laser_data;
    for(int i = 0; i < laser_positions.size(); ++i){
        laser_data.push_back(std::make_tuple(i, (laser_positions[i] - current_pos).rotated(-curr_yaw).angle(), (laser_positions[i] - current_pos).norm()));
    }

    std::vector<double> pure_angles(laser_data.size());
    std::transform(laser_data.begin(), laser_data.end(), pure_angles.begin(), [](std::tuple<int, double, double> el){return std::get<1>(el);});

    auto min_rel_angle = *std::min_element(pure_angles.begin(), pure_angles.end());
    auto max_rel_angle = *std::max_element(pure_angles.begin(), pure_angles.end());

    std::vector<std::vector<int>> laser_segment_idxs;
    auto angle_step = (max_rel_angle - min_rel_angle) / this->segments;
    for(int i = 0; i < this->segments; ++i){
        std::vector<int> segment_idxs;
        for(auto laser_angle : laser_data){
            if(std::get<1>(laser_angle) >= min_rel_angle + angle_step * i && std::get<1>(laser_angle) <= min_rel_angle + angle_step * (i + 1)){
                segment_idxs.push_back(std::get<0>(laser_angle));
            }
        }
        laser_segment_idxs.push_back(segment_idxs);
    }


    // for(auto segment : laser_segment_idxs){
    //     std::cout << "Segment size: " << segment.size() << std::endl;
    // }
    std::cout << laser_segment_idxs.size() << std::endl;

    std::vector<Vec2f> closest_elements;

    for(auto segment_idxs : laser_segment_idxs){
        auto closest_idx = *std::min_element(
            segment_idxs.begin(),
            segment_idxs.end(),
            [&](const int& idx_a, const int& idx_b) {
                return compare_func_idx_based(idx_a) < compare_func_idx_based(idx_b);
            }
        );
        double rho_q = std::get<2>(laser_data[closest_idx]);
        if(rho_q > this->rho_0){
            closest_elements.push_back(Vec2f(0, 0));
        }else{  
            auto vector = this->k_rep * ((current_pos - laser_positions[closest_idx]) * (1.0 / rho_q)) * (1.0 / rho_q - 1.0 / rho_0) * (1.0 / std::pow(rho_q, 2));
            closest_elements.push_back(vector);
        }
        
    }

    return closest_elements;
   
}

Vec2f PotentialField::get_total_force(std::vector<Vec2f> f_reps, Vec2f f_att){
    Vec2f f_tot = f_att;

    for(auto f_rep : f_reps){
        f_tot += f_rep;
    }

    return f_tot;
}