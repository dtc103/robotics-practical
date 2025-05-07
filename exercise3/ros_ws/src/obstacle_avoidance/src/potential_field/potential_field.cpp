#include <potential_field/potential_field.h>


PotentialField::PotentialField(Vec2f goal_position, double k_att, double k_rep, size_t segments):goal_position(goal_position), k_att(k_att), k_rep(k_rep), segments(segments){
    
}
Vec2f PotentialField::get_f_att(Vec2f current_position){
    return -this->k_att * (current_position - this->goal_position);
}

std::vector<Vec2f> PotentialField::get_f_rep(Vec2f current_pos, std::vector<Vec2f> laser_positions){
    size_t totalSize = laser_positions.size();
    std::vector<std::vector<Vec2f>> result(this->segments);

    auto min_rel_angle = -std::numbers::pi / 2;
    auto max_rel_angle = std::numbers::pi / 2;

    std::vector<std::tuple<int, double>> laser_angles;
    for(auto pos : laser_positions){
            
    }

    std::vector<Vec2f> closest_elements;
    auto map_func = [&](const Vec2f& a){return (a - current_pos).norm();};
    for(auto segment : result){
        closest_elements.push_back(*std::min_element(
            segment.begin(),
            segment.end(),
            [&](const Vec2f& a, const Vec2f& b) {
                return map_func(a) < map_func(b);
            }
        ));
    }

    return closest_elements;


    
}

Vec2f PotentialField::get_total_force(Vec2f current_position, std::vector<Vec2f> laser_positions){
    return Vec2f(0,0);
}