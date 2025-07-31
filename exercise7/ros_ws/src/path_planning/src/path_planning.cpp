#include "path_planning.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

PathPlanning::PathPlanning(): rclcpp::Node("path_planning") {
    this->declare_parameter<double>("threshold", 60.0);
    this->declare_parameter<double>("inflation_radius", 1.5);
    this->declare_parameter<double>("sigma", 1.5);

    inflation_radius_ = this->get_parameter("inflation_radius").as_double();
    threshold = this->get_parameter("threshold").as_double();
    sigma = this->get_parameter("sigma").as_double();

    this->path.header.frame_id = "odom";

    std::cout << "Initialized parameters" << std::endl;

    this->timer = this->create_wall_timer(1000ms, std::bind(&PathPlanning::timer_callback, this));

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
    //std::cout << "POSITION ->" << odom.pose.pose.position.x << ":" << odom.pose.pose.position.y << std::endl;
    if(!start_position_recorded){
        start_position.x = odom.pose.pose.position.x;
        start_position.y = odom.pose.pose.position.y;

        std::cout << "START POSITION ->" << start_position.x << ":" << start_position.y << std::endl;

        this->start_position_recorded = true;
    }
}

void PathPlanning::goal_callback(const geometry_msgs::msg::PoseStamped &goal)
{
    //std::cout << "POSITION ->" << odom.pose.pose.position.x << ":" << odom.pose.pose.position.y << std::endl;
    goal_position.x = goal.pose.position.x;
    goal_position.y = goal.pose.position.y;

    std::cout << "NEW GOAL POSITION ->" << goal_position.x << ":" << goal_position.y << std::endl;

    this->path_calculated = false;
    this->start_position_recorded = false;
}

void PathPlanning::grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    std::cout << "GRID CALLBACK" << std::endl;

    this->grid = *msg;

    publishCostMap();

    if(!this->path_calculated && this->start_position_recorded){
        std::cout << "Start calculating route" << std::endl;
        plan_route(this->start_position, this->goal_position);

        std::cout << "Finished calculating path" << std::endl;

        this->path_calculated = true;
    }
}

void PathPlanning::create_cost_map() {
    this->get_parameter("inflation_radius", inflation_radius_);
    this->get_parameter("sigma", sigma);

    auto start = std::chrono::high_resolution_clock::now();
  
    int w = grid.info.width;
    int h = grid.info.height;
    int N = w * h;
    double res = grid.info.resolution;
    double R   = inflation_radius_;
  
    std::vector<double> f(N, std::numeric_limits<double>::infinity());
    for(int i = 0; i < N; ++i) {
      if(grid.data[i] > threshold) {
        f[i] = 0.0;
      }
    }
  
    // 2) EDT in O(N)
    std::vector<double> dist2(N);
    edt_2d(f, dist2, w, h);
  
    double rad_cells = R / res;
    double rad2 = rad_cells * rad_cells;
    for(int i = 0; i < N; ++i) {
      if(dist2[i] > rad2) dist2[i] = std::numeric_limits<double>::infinity();
    }
  
    applyLinear(dist2, res, R);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;

    std::cout << elapsed.count() << std::endl;
  }
  

void PathPlanning::publishCostMap() {
    create_cost_map();

    cost_grid = grid;  
    cost_grid.header.stamp = this->get_clock()->now();
    size_t N = cost_map_.size();
    cost_grid.data.resize(N);
    for(size_t i = 0; i < N; ++i){
      int v = static_cast<int>(std::round(cost_map_[i] * 100.0));
      if (grid.data[i] > threshold) v = 100;  
      cost_grid.data[i] = static_cast<int8_t>(v);
    }

    costMapPublisher->publish(cost_grid);
}


void PathPlanning::plan_route(Vec2f start, Vec2f goal){
    int start_idx = get_grid_index(start);
    int goal_idx = get_grid_index(goal);

    publishCostMap();
    

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
    std::priority_queue<Node, std::vector<Node>, Compare> open;
    open.push({start, this->shapley_distance(start, goal, width), 0});

    std::unordered_map<int, int> came_from;
    std::unordered_map<int, double> g_score;
    g_score[start] = 0.0;

    std::unordered_set<int> closed;

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
                std::cout << "FKLJDGBHSFJKDVÖBLJ VFBJKLDS BIULND FKJÖDN L!" << std::endl;
                path.push_back(cur);
                if (cur == start)
                    break;
                cur = came_from[cur];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        if (closed.count(current.idx))
            continue;
        closed.insert(current.idx);

        int cx = current.idx % width;
        int cy = current.idx / width;
        // 4-neighborhood
        const int dirs[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        for (auto &d : dirs)
        {
            int nx = cx + d[0];
            int ny = cy + d[1];
            if (nx < 0 || nx >= height || ny < 0 || ny >= width)
                continue;
            int nidx = ny * width + nx;
            if (this->grid.data[nidx] > this->threshold)
                continue; // occupied
            
            double base = (std::abs(d[0]) + std::abs(d[1]) == 2)
            ? std::sqrt(2.0)
            : 1.0;
            double move_cost = base + cost_map_[nidx];
            double tentative_g = current.g + move_cost;

            auto it = g_score.find(nidx);
            if (it == g_score.end() || tentative_g < it->second)
            {
                came_from[nidx] = current.idx;
                g_score[nidx] = tentative_g;
                double f = tentative_g + shapley_distance(nidx, goal, width);
                open.push({nidx, f, tentative_g});
            }
        }
    }

    return {}; // no path found
}

void PathPlanning::timer_callback(){
    std::cout << this->path_calculated << ", " << this->start_position_recorded << std::endl;
    if(this->path_calculated){
        for(auto p : this->path.poses){
            std::cout << p.pose.position.x << ":" << p.pose.position.y << " | ";
        }
        std::cout << std::endl;
        this->pathPublisher->publish(this->path);
    }
}

Vec2i PathPlanning::odom_to_grid(Vec2f world) {
    double gx = (world.x - this->grid.info.origin.position.x) / this->grid.info.resolution;
    double gy = (world.y - this->grid.info.origin.position.y) / this->grid.info.resolution;
    return Vec2i(gx, gy);
}

int PathPlanning::get_grid_index(Vec2f &p_world){
    Vec2i g = odom_to_grid(p_world);

    if(g.x < 0 || g.x >= this->grid.info.width || g.y < 0 || g.y >= this->grid.info.height){
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


void PathPlanning::applyGaussian(const std::vector<double>& dist2, double resolution, double sigma, double inflation_radius) {
    int N = dist2.size();
    double twoSigma2 = 2.0 * sigma * sigma;

    for(int i=0; i<N; ++i){
        if(dist2[i] < std::numeric_limits<double>::infinity()){
            double dist_m = std::sqrt(dist2[i]) * resolution;
            if(dist_m <= inflation_radius) {
                cost_map_[i] = 2.0 * std::exp(-(dist_m*dist_m) / twoSigma2);
                continue;
            }
        }
        cost_map_[i] = 0.0;
    }
}


void PathPlanning::applyLinear(const std::vector<double>& dist2, double resolution, double inflation_radius) {
    int N = dist2.size();
    for(int i = 0; i < N; ++i) {
        if(dist2[i] < std::numeric_limits<double>::infinity()) {
            double dist_m = std::sqrt(dist2[i]) * resolution;
            if(dist_m <= inflation_radius) {
                cost_map_[i] = (inflation_radius - dist_m) / inflation_radius;
                continue;
            }
        }
        cost_map_[i] = 0.0;
    }
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



// Fast Euclidean Distance Transform (Felzenszwalb & Huttenlocher):
// edt_1d: 1D pass computing min (q–p)² + f[p] for a vector f in O(n).
// edt_2d: applies edt_1d to each column then each row to get
//         squared distance-to-obstacle for every grid cell in O(width·height).
void PathPlanning::edt_1d(const std::vector<double>& f, std::vector<double>& d, int n) {
    std::vector<int> v(n);
    std::vector<double> z(n+1);
    const double INF_D = std::numeric_limits<double>::infinity();
    int k = 0;
    v[0] = 0;
    z[0] = -INF_D;
    z[1] = +INF_D;
    for(int q = 1; q < n; ++q) {
      double s = ((f[q] + q*q) - (f[v[k]] + v[k]*v[k])) / (2.0*(q - v[k]));
      while(s <= z[k]) {
        --k;
        s = ((f[q] + q*q) - (f[v[k]] + v[k]*v[k])) / (2.0*(q - v[k]));
      }
      ++k; 
      v[k] = q;
      z[k]   = s;
      z[k+1] = +INF_D;
    }
    k = 0;
    for(int q = 0; q < n; ++q) {
      while(z[k+1] < q) ++k;
      double diff = q - v[k];
      d[q] = diff*diff + f[v[k]];
    }
  }
  
void PathPlanning::edt_2d(const std::vector<double>& grid, std::vector<double>& dist2, int w, int h) {
    std::vector<double> tmp(std::max(w,h));
    for(int x = 0; x < w; ++x) {
      for(int y = 0; y < h; ++y)
        tmp[y] = grid[y*w + x];
      std::vector<double> col_d(h);
      edt_1d(tmp, col_d, h);
      for(int y = 0; y < h; ++y)
        dist2[y*w + x] = col_d[y];
    }
    for(int y = 0; y < h; ++y) {
      for(int x = 0; x < w; ++x)
        tmp[x] = dist2[y*w + x];
      std::vector<double> row_d(w);
      edt_1d(tmp, row_d, w);
      for(int x = 0; x < w; ++x)
        dist2[y*w + x] = row_d[x];
    }
  }

  