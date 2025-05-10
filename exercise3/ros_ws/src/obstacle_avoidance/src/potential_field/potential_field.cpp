#include <potential_field/potential_field.h>

PotentialField::PotentialField(Vec2f goal_position, double k_att, double k_rep, double var_rep, double rho_0, size_t segments) 
    : goal_position(goal_position), k_att(k_att), k_rep(k_rep), var_rep(var_rep), rho_0(rho_0), segments(segments){
    
}

Vec2f PotentialField::get_f_att(Vec2f current_position){
    auto vec = -this->k_att * (current_position - this->goal_position);
    if(vec.norm() < 1.0){
        vec = -10.0 * (current_position - this->goal_position);
    }
    return vec;
}

std::vector<std::tuple<int, Vec2f>> PotentialField::get_f_rep(Vec2f current_pos, double curr_yaw, std::vector<Vec2f> laser_positions){
    // save index of lasaer_positions, corresponding angles and norm

    std::vector<std::tuple<int, double, double>> laser_data;
    double max_norm = 10000.0;
    for(int i = 0; i < laser_positions.size(); ++i){
        laser_data.push_back(
            std::make_tuple(
                i, 
                (laser_positions[i] - current_pos).rotated(-curr_yaw).angle(), 
                (laser_positions[i] - current_pos).norm() < max_norm ? (laser_positions[i] - current_pos).norm() : max_norm
            )
        );
    }

    // get angle of most left laser beam
    auto min_rel_angle = std::get<1>(
        *std::min_element(
            laser_data.begin(), 
            laser_data.end(), 
            [](auto a, auto b){
                return std::get<1>(a) < std::get<1>(b);
            }
        )
    );

    // get angle of most right laser beam
    auto max_rel_angle = std::get<1>(
        *std::max_element(
            laser_data.begin(), 
            laser_data.end(), 
            [](auto a, auto b){
                return std::get<1>(a) < std::get<1>(b);
            }
        )
    );

    std::vector<std::vector<int>> laser_segment_idxs;
    auto angle_step = (max_rel_angle - min_rel_angle) / this->segments;
    for(size_t i = 0; i < this->segments; ++i){
        std::vector<int> segment_idxs;
        for(auto laser_angle : laser_data){
            if(std::get<1>(laser_angle) >= min_rel_angle + angle_step * i && std::get<1>(laser_angle) < min_rel_angle + angle_step * (i + 1)){
                segment_idxs.push_back(std::get<0>(laser_angle));
            }
        }
        laser_segment_idxs.push_back(segment_idxs);
    }

    std::vector<std::tuple<int, Vec2f>> closest_elements;
    for(auto segment_idxs : laser_segment_idxs){
        if(segment_idxs.empty()){
            continue;
        }

        // returns the index for the closes laser beam
        auto closest_idx = *std::min_element(
            segment_idxs.begin(),
            segment_idxs.end(),
            [&](const int& idx_a, const int& idx_b) {
                return std::get<2>(laser_data[idx_a]) < std::get<2>(laser_data[idx_b]);
            }
        );
        
        double rho_q = std::get<2>(laser_data[closest_idx]);
        if(rho_q > this->rho_0){
            continue;
        }else{  
            // we also divided the k_rep by the amount of sesgements, so more segments don't start adding up the kRep attribute
            auto vector =  (this->rotational_scaling(std::get<1>(laser_data[closest_idx])) / this->segments) * (1.0 / rho_q - 1.0 / rho_0) * (1.0 / std::pow(rho_q, 2)) * ((current_pos - laser_positions[closest_idx]) * (1.0 / rho_q)) ;
            closest_elements.push_back(std::make_tuple(closest_idx, vector));
        }
    }

    return closest_elements;
   
}

Vec2f PotentialField::get_total_force(std::vector<std::tuple<int, Vec2f>> f_reps, Vec2f f_att){
    Vec2f f_tot = f_att;

    for(auto f_rep : f_reps){
        f_tot += std::get<1>(f_rep);
    }

    return f_tot;
}

double PotentialField::rotational_scaling(double angle){
    double a = this->k_rep;
    double b = this->var_rep;

    return a * std::exp(-b * std::pow(angle, 2));
}