#include <potential_field/potential_field.h>


PotentialField::PotentialField(Vec2f goal_position, double k_att, double k_rep, size_t segments):goal_position(goal_position), k_att(k_att), k_rep(k_rep), segments(segments){
    
}
Vec2f PotentialField::get_f_att(Vec2f current_position){
    return -this->k_att * (current_position - this->goal_position);
}

std::vector<Vec2f> PotentialField::get_f_rep(Vec2f current_pos, std::vector<Vec2f> laser_positions){
    size_t totalSize = laser_positions.size();
    std::vector<std::vector<Vec2f>> result(this->segments);
    
    size_t baseSize = totalSize / this->segments;
    size_t remainder = totalSize % this->segments;
    
    auto it = laser_positions.begin();
    for (size_t i = 0; i < this->segments; ++i) {
        size_t segmentSize = baseSize + (i < remainder ? 1 : 0);
        result[i] = std::vector<Vec2f>(it, it + segmentSize);
        it += segmentSize;
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

    for (auto el : closest_elements){
        std::cout << el.x << " " << el.y << std::endl;
    }


    
}

Vec2f PotentialField::get_total_force(Vec2f current_position, std::vector<Vec2f> laser_positions){
    return Vec2f(0,0);
}