#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cassert>
#include "path_processing.h"

nav_msgs::msg::Path processPath(const nav_msgs::msg::Path &path) {
    std::vector<double> p_;
    std::vector<double> q_;
    std::vector<double> p_prim_;
    std::vector<double> q_prim_;

    nav_msgs::msg::Path path_interp_;

    int N = path.poses.size();
    std::cout << "The original path has " << N << " poses." << std::endl;

    if (N < 2)
        return path_interp_;

    // first, copy the path waypoints to X_arr and Y_arr, and calculate the distance vector l_arr
    double X_arr[N], Y_arr[N], l_arr[N], l_arr_unif[N];
    double L = 0;
    geometry_msgs::msg::PoseStamped first_wp = path.poses.at(0);

    X_arr[0] = first_wp.pose.position.x;
    Y_arr[0] = first_wp.pose.position.y;
    l_arr[0] = 0;

    int insert_index = 1;
    for (std::size_t i = 1; i < N; i++) {
        geometry_msgs::msg::PoseStamped wp = path.poses.at(i);

        double dist = hypot(wp.pose.position.x - X_arr[insert_index-1], wp.pose.position.y - Y_arr[insert_index-1]);
        if (dist >= 1e-3) {
            X_arr[insert_index] = wp.pose.position.x;
            Y_arr[insert_index] = wp.pose.position.y;

            L += dist;
            l_arr[insert_index] = L;

            ++insert_index;
        } else {
            // two points were to close...
            std::cout << "dropping point (" << wp.pose.position.x << " / " << wp.pose.position.y <<
                ") because it is too close to the last point (" << X_arr[insert_index-1] << " / " << Y_arr[insert_index-1] << ")" << std::endl;
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("path_processing"), "Length of the unsmoothed path: %lf m", L);

    // calculate the arclength vector
    double f = std::max(0.001, L / (double) (N-1));
    for (std::size_t i = 0; i < N; i++)
        l_arr_unif[i] = i * f;

    // copy the X_arr, Y_arr, l_arr and l_arr_unif to alglib arrays
    alglib::real_1d_array X_alg, Y_alg, l_alg, l_alg_unif;
    alglib::real_1d_array x_s, y_s, x_s_prim, y_s_prim, x_s_sek, y_s_sek;

    X_alg.setcontent(N, X_arr);
    Y_alg.setcontent(N, Y_arr);
    l_alg.setcontent(N, l_arr);
    l_alg_unif.setcontent(N, l_arr_unif);

    // interpolate the path and find the derivatives
    // l_alg is the old base, and l_alg_unif the new one (uniform arclength)
    try {
        alglib::spline1dconvdiff2cubic(l_alg, X_alg, l_alg_unif, x_s, x_s_prim, x_s_sek);
        alglib::spline1dconvdiff2cubic(l_alg, Y_alg, l_alg_unif, y_s, y_s_prim, y_s_sek);
    } catch(const alglib::ap_error& error) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("path_processing"), "alglib error: " << error.msg);
        throw error;
    }

    // smoothen the path
    alglib::spline1dinterpolant s1, s2;
    alglib::spline1dfitreport rep1, rep2;
    double rho = 1.7;
    alglib::ae_int_t info1, info2;
    alglib::real_1d_array x_sm, y_sm, l_alg_sm;

    // initialize the vectors x_sm and y_sm
    double h_arr[N], l_arr_sm[N];
    for (std::size_t i = 0; i < N; i++)
        h_arr[i] = 0.0;

    x_sm.setcontent(N, h_arr);
    y_sm.setcontent(N, h_arr);

    // calculate the smoothed spline interpolants s1 and s2
    try {
        alglib::spline1dfitpenalized(l_alg_unif, x_s, N, rho, info1, s1, rep1);
        alglib::spline1dfitpenalized(l_alg_unif, y_s, N, rho, info2, s2, rep2);
    } catch(const alglib::ap_error& error) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("path_processing"), "alglib error: " << error.msg);
        throw error;
    }

    // use the interpolants and the base of the interpolated path l_alg_unif to calculate
    // the smooth values x_sm and y_sm
    try {
        x_sm[0] = alglib::spline1dcalc(s1, l_alg_unif[0]);
        y_sm[0] = alglib::spline1dcalc(s2, l_alg_unif[0]);
        double L_sm = 0.0;

        for (std::size_t i = 1; i < N; i++) {
            x_sm[i] = alglib::spline1dcalc(s1, l_alg_unif[i]);
            y_sm[i] = alglib::spline1dcalc(s2, l_alg_unif[i]);

            double dist = hypot(x_sm[i] - x_sm[i-1], y_sm[i] - y_sm[i-1]);

            L_sm += dist;
            l_arr_sm[i] = L_sm;
        }
        RCLCPP_INFO(rclcpp::get_logger("path_processing"), "Length of the smoothed path: %lf m", L_sm);
    } catch(const alglib::ap_error& error) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("path_processing"), "alglib error: " << error.msg);
        throw error;
    }

    l_alg_sm.setcontent(N, l_arr_sm);

    // interpolate over a new grid l_arr_unif_sm[M]
    // this new grid is the arclength vector of the smoothed path, but with a different number of points (M)
    int M = 3*N;
    //std::cout << "The interpolated path has " << M << " poses." << std::endl;

    double l_arr_unif_sm[M];
    double h = std::max(0.001, L / (double) (M-1));

    for (std::size_t i = 0; i < M; i++)
        l_arr_unif_sm[i] = i * h;

    alglib::real_1d_array l_alg_unif_sm;
    l_alg_unif_sm.setcontent(M, l_arr_unif_sm);

    try {
        alglib::spline1dconvdiff2cubic(l_alg_unif, x_sm, l_alg_unif_sm, x_s, x_s_prim, x_s_sek);
        alglib::spline1dconvdiff2cubic(l_alg_unif, y_sm, l_alg_unif_sm, y_s, y_s_prim, y_s_sek);
    } catch(const alglib::ap_error& error) {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("path_processing"), "alglib error: " << error.msg);
        throw error;
    }

    // define path components, its derivatives, and curvilinear abscissa, then calculate the path curvature
    for (uint i = 0; i < M; ++i) {
        p_.push_back(x_s[i]);
        q_.push_back(y_s[i]);

        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = x_s[i];
        pose.pose.position.y = y_s[i];
        pose.header = path.header;
        //pose.header.seq = i;

        path_interp_.poses.push_back(pose);
        path_interp_.header = path.header;

        p_prim_.push_back(x_s_prim[i]);
        q_prim_.push_back(y_s_prim[i]);
    }

    assert(p_prim_.size() == M);
    assert(q_prim_.size() == M);
    assert(p_.size() == M);
    assert(q_.size() == M);

    return path_interp_;
}
