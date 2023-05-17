#include <iostream>
#include <algorithm>
#include <stack>
#include <cmath>
#include <vector>
#include <iomanip>

struct Point {
    Point() = default;
    
    Point(int64_t x, int64_t y) : x_(x), y_(y) {}

    Point operator-(const Point& b) {
        return {x_ - b.x_, y_ - b.y_};
    }

    int64_t x_;
    int64_t y_;
};

int64_t Collinear(Point a, Point b) {
    return a.x_ * b.y_ - a.y_ * b.x_;
}

std::vector<Point> ConvexHull(std::vector<Point>& points, size_t n) {
    
    sort(points.begin(), points.end(), [&](Point& a, Point& b) {
        return a.x_ < b.x_ || (a.x_ == b.x_ && a.y_ < b.y_);
    });

    std::vector<Point> lower_half;
    lower_half.push_back(points[0]);

    std::vector<Point> upper_half;
    upper_half.push_back(points[0]);

    for(size_t i = 1; i < n; ++i) {
        
        //delete points lying on the same line
        while(lower_half.size() > 1) {
            Point last_point = *(lower_half.end() - 1);
            Point prelast_point = *(lower_half.end() - 2);
            if (Collinear(last_point - prelast_point, points[i] - last_point) <= 0) {
                lower_half.pop_back();
            }
            else {
                break;
            }
        }

        while(upper_half.size() > 1) {
            Point last_point = *(upper_half.end() - 1);
            Point prelast_point = *(upper_half.end() - 2);
            if (Collinear(last_point - prelast_point, points[i] - last_point) >= 0) {
                upper_half.pop_back();
            }
            else {
                break;
            }
        }

        lower_half.push_back(points[i]);
        upper_half.push_back(points[i]);
    }

    std::vector<Point> convex_hull;

    for(auto point : upper_half) {
        convex_hull.push_back(point);
    }

    lower_half.pop_back();
    reverse(lower_half.begin(), lower_half.end());
    lower_half.pop_back();

    for (auto point : lower_half) {
        convex_hull.push_back(point);
    }

    return convex_hull;
    
}

double Square(std::vector<Point>& points, size_t n) {
    int64_t area = 0.0;
    size_t j = n - 1;
    for(size_t i = 0; i < n; ++i) {
        area += (points[j].x_ - points[i].x_) * (points[j].y_ + points[i].y_);
        j = i;
    }
    return static_cast<double>(std::abs(area)) / 2.0;
}

int main() {
    size_t n = 0;
    std::cin >> n;
    std::vector<Point> points(n);
    for(size_t i = 0; i < n; ++i) {
        std::cin >> points[i].x_ >> points[i].y_;
    }
    std::vector<Point> convex_hull = ConvexHull(points, n);
    std::cout << convex_hull.size() << '\n';
    for(auto point : convex_hull) {
        std::cout << point.x_ << " " << point.y_ << '\n';
    }
    std::cout << std::fixed << std::setprecision(1) << Square(convex_hull, convex_hull.size());

    return 0;

}