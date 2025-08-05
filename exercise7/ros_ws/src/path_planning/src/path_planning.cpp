#include "path_planning.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

PathPlanning::PathPlanning(): rclcpp::Node("path_planning") {
    this->declare_parameter<double>("threshold", 60.0);
    this->declare_parameter<double>("inflation_radius", 1.5);
    this->declare_parameter<double>("sigma", 1.5);
    this->declare_parameter<double>("min_clearance", 0.5);


    inflation_radius_ = this->get_parameter("inflation_radius").as_double();
    threshold = this->get_parameter("threshold").as_double();
    sigma = this->get_parameter("sigma").as_double();
    min_clearance = this->get_parameter("min_clearance").as_double();


    this->path.header.frame_id = "odom";

    std::cout << "Initialized parameters" << std::endl;

    this->timer = this->create_wall_timer(100ms, std::bind(&PathPlanning::timer_callback, this));

    std::cout << "Created timer" << std::endl;

    subOdom = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 1,
        std::bind(&PathPlanning::odomCallback, this, std::placeholders::_1));

    this->pathPublisher = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    this->costMapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/cost_map", 10);

    gridSubscription = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/grid", 10, std::bind(&PathPlanning::grid_callback, this, _1));

    std::cout << "Created publishers and subscribers" << std::endl;

    goalSubscription = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&PathPlanning::goal_callback, this, _1));
}

void PathPlanning::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    current_position_.x = odom.pose.pose.position.x;
    current_position_.y = odom.pose.pose.position.y;
    odom_received_ = true;
}

void PathPlanning::goal_callback(const geometry_msgs::msg::PoseStamped &goal)
{
    //std::cout << "POSITION ->" << odom.pose.pose.position.x << ":" << odom.pose.pose.position.y << std::endl;
    goal_position.x = goal.pose.position.x;
    goal_position.y = goal.pose.position.y;

    std::cout << "NEW GOAL POSITION ->" << goal_position.x << ":" << goal_position.y << std::endl;

    this->goal_active_ = true;
}

void PathPlanning::grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    //std::cout << "GRID CALLBACK" << std::endl;

    this->grid = *msg;

    publish_cost_map();
}

void PathPlanning::create_cost_map(){
    this->get_parameter("inflation_radius", inflation_radius_);
    this->get_parameter("sigma", sigma);

    int w = grid.info.width;
    int h = grid.info.height;
    int N = w * h;
    double res = grid.info.resolution;
    double R = inflation_radius_;
    double rad_cells = R / res;
    double rad2 = rad_cells * rad_cells;



    auto t_start = std::chrono::high_resolution_clock::now();

    dist2_ = computeDistanceMap();

    auto t_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

    std::cout << "Costmap calculation:" <<  duration << " ms" << std::endl;

    // prepare
    cost_map_.assign(N, 0.0);

    
    // 3) apply a one-sided Gaussian: cost = exp(–(d²)/(2σ²)), with σ=R/2
    double twoSigma2 = 2.0 * sigma * sigma;
    for(int i = 0; i < N; ++i){
      if(dist2_[i] <= rad2){
        // convert cell-units back to meters
        double d_m = std::sqrt(dist2_[i]) * res;
        cost_map_[i] = 2 * std::exp( - (d_m * d_m) / twoSigma2);
      }
      // beyond R: cost_map_[i] stays 0
    }
}

void PathPlanning::plan_route(Vec2f start, Vec2f goal){
    int start_idx = get_grid_index(start);
    int goal_idx = get_grid_index(goal);

    auto path = astar(start_idx, goal_idx, this->grid.info.width, this->grid.info.height);
    std::vector<geometry_msgs::msg::PoseStamped> world_coords;
    for(auto idx : path) {
        Vec2f world_coord = gridIndexToWorld(idx);
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = world_coord.x;
        pose.pose.position.y = world_coord.y;
        world_coords.push_back(pose);
    }

    this->path.poses = world_coords;
}

std::vector<int> PathPlanning::astar(int start, int goal, int width, int height)
{   

    int N = height * width;
    std::priority_queue<Node, std::vector<Node>, Compare> open;
    open.push({start, this->shapley_distance(start, goal, width), 0});
    std::vector<double> g_score(N, std::numeric_limits<double>::infinity());
    std::vector<int>    came_from(N, -1);
    std::vector<uint8_t> closed(N, 0);
    g_score[start] = 0.0;

    double min_clearance_cells = min_clearance / grid.info.resolution;

    while (!open.empty())
    {
        Node current = open.top();
        open.pop();
        if (current.idx == goal)
        {
            // reconstruct path
            std::vector<int> path;
            int cur = goal;
            while (true)
            {
                path.push_back(cur);
                if (cur == start)
                    break;
                cur = came_from[cur];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (closed[current.idx]) 
            continue;
        closed[current.idx] = 1;

        int cx = current.idx % width;
        int cy = current.idx / width;
        // 4-neighborhood
        const int dirs[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        for (auto &d : dirs)
        {
            int nx = cx + d[0];
            int ny = cy + d[1];

            int nidx = ny * width + nx;

            // check bounds
            if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                continue;

            // gap to small
            if (dist2_[nidx] < min_clearance_cells / 2) {
                //std::cout << dist2_[nidx] << std::endl;
                continue;
            }
            
            // avoids corner cutting
            if (std::abs(d[0])==1 && std::abs(d[1])==1) {
                int idx_x = cy*width + (cx + d[0]);      // horizontal neighbour
                int idx_y = (cy + d[1])*width + cx;      // vertical neighbour
                if (grid.data[idx_x] > threshold || grid.data[idx_y] > threshold)
                    continue;
                }

            // skip occupied cells or unknown cells  
            if (this->grid.data[nidx] > this->threshold || grid.data[nidx] == 50)
                continue; 
             
            
            double base = (std::abs(d[0]) + std::abs(d[1]) == 2)
            ? std::sqrt(2.0)
            : 1.0;
            double move_cost = base + cost_map_[nidx];
            double tentative_g = current.g + move_cost;



            if (tentative_g < g_score[nidx]) {
                came_from[nidx] = current.idx;
                g_score[nidx]    = tentative_g;

                double h = shapley_distance(nidx, goal, width);
                double f = tentative_g + h;

                open.push({ nidx, f, tentative_g });
            }
        }
    }

    return {}; // no path found
}

void PathPlanning::timer_callback(){
    if (odom_received_ && goal_active_) {
        auto t_start = std::chrono::high_resolution_clock::now();

        plan_route(current_position_, goal_position);

        this->path.header.stamp = this->get_clock()->now();
        this->pathPublisher->publish(this->path);

        auto t_end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

        std::cout << "Replanned path from ("
        << current_position_.x << ", " << current_position_.y
        << ") to ("
        << goal_position.x << ", " << goal_position.y
        << ") in " << duration << " ms" << std::endl;

    }
}

Vec2i PathPlanning::odom_to_grid(Vec2f world) {
    double gx = (world.x - this->grid.info.origin.position.x) / this->grid.info.resolution;
    double gy = (world.y - this->grid.info.origin.position.y) / this->grid.info.resolution;
    return Vec2i(gx, gy);
}

int PathPlanning::get_grid_index(Vec2f &p_world){
    Vec2i g = odom_to_grid(p_world);

    if(g.x < 0 || g.x >= (int) this->grid.info.width || g.y < 0 || g.y >= (int) this->grid.info.height){
        return -1;
    }

    return g.y * this->grid.info.width + g.x;
}

Vec2f PathPlanning::grid_coords(int idx) {
    int x = idx % this->grid.info.width;
    int y = idx / this->grid.info.width;
    return Vec2f(x, y);
}

Vec2f PathPlanning::gridIndexToWorld(int idx) {
    int w = this->grid.info.width;
    int h = this->grid.info.height;
    int max_cells = w * h;

    if (idx < 0 || idx >= max_cells) {
        return Vec2f(-1, -1);  // invalid index
    }

    // recover integer cell‐coords
    int i = idx % w;
    int j = idx / w;

    // bottom‐left corner of the grid in odom frame:
    double ox = this->grid.info.origin.position.x;
    double oy = this->grid.info.origin.position.y;
    double r  = this->grid.info.resolution;

    double x_world = ox + (i + 0.5) * r;
    double y_world = oy + (j + 0.5) * r;

    return Vec2f(x_world, y_world);
}

double PathPlanning::manhatten_distance(int idx, int goal, int width) {
    int x1 = idx / width;
    int y1 = idx % width;
    int x2 = goal / width;
    int y2 = goal % width;
    return static_cast<double>(abs(x1 - x2) + abs(y1 - y2));
}

double PathPlanning::shapley_distance(int a, int b, int width) {
    int x1 =  a % width, y1 = a / width;
    int x2 =  b % width, y2 = b / width;
    int dx = std::abs(x1 - x2);
    int dy = std::abs(y1 - y2);
    // min(dx,dy) Schritte diagonal (√2), Rest gerade (1.0)
    return std::min(dx,dy) * std::sqrt(2.0) + std::abs(dx - dy) * 1.0;
}


void PathPlanning::publish_cost_map() {
    create_cost_map();

    cost_grid = grid;
    cost_grid.header.stamp = this->get_clock()->now();

    size_t N = cost_map_.size();
    cost_grid.data.resize(N);
    for (size_t i = 0; i < N; ++i) {
        int v = static_cast<int>(std::round(cost_map_[i] * 100.0));
        if (grid.data[i] > threshold) v = 100;
        cost_grid.data[i] = static_cast<int8_t>(v);
    }

    costMapPublisher->publish(cost_grid);
}


std::vector<float> PathPlanning::computeDistanceMap() {
    int w = grid.info.width;
    int h = grid.info.height;
    int N = w * h;
    float res = grid.info.resolution;
    float R = inflation_radius_;
    float rad_cells = R / res;
    float rad2 = rad_cells * rad_cells;

    std::vector<float> dist2(N, std::numeric_limits<float>::infinity());
    using PQE = std::pair<float,int>;
    std::priority_queue<PQE, std::vector<PQE>, std::greater<>> pq;

    // 1) Seed obstacles
    for (int i = 0; i < N; ++i) {
        if (grid.data[i] > threshold) {
            dist2[i] = 0.0;
            pq.push({0.0, i});
        }
    }

    // 2) Multi-source Dijkstra up to radius
    const int dirs[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    while (!pq.empty()) {
        auto [d2, idx] = pq.top();
        pq.pop();
        if (d2 > dist2[idx] || d2 > rad2) continue;
        int x = idx % w;
        int y = idx / w;
        for (auto &d : dirs) {
            int nx = x + d[0];
            int ny = y + d[1];
            if (nx<0||nx>=w||ny<0||ny>=h) continue;
            int ni = ny*w + nx;
            float step2 = (std::abs(d[0])+std::abs(d[1])==2 ? std::sqrt(2.0) : 1.0);
            float nd2 = d2 + step2;
            if (nd2 < dist2[ni] && nd2 <= rad2) {
                dist2[ni] = nd2;
                pq.push({nd2, ni});
            }
        }
    }
    return dist2;
}

