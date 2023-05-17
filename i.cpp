#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>

struct ThreeDimensionalVector {
    ThreeDimensionalVector() { 
        Coord[0] = Coord[1] = Coord[2] = 0; 
    }

    ThreeDimensionalVector(int64_t x, int64_t y, int64_t z) { 
        Coord[0] = x; 
        Coord[1] = y; 
        Coord[2] = z; 
    }

    ThreeDimensionalVector operator*(const ThreeDimensionalVector& v) const {
        return {Coord[1] * v.Coord[2] - Coord[2] * v.Coord[1],
                    Coord[2] * v.Coord[0] - Coord[0] * v.Coord[2],
                    Coord[0] * v.Coord[1] - Coord[1] * v.Coord[0]};
    }

    ThreeDimensionalVector operator-(const ThreeDimensionalVector& v) const {
        return {Coord[0] - v.Coord[0], Coord[1] - v.Coord[1], Coord[2] - v.Coord[2]};
    }

    ThreeDimensionalVector operator-() const {
        return {-Coord[0], -Coord[1], -Coord[2]};
    }

    int64_t Dot(const ThreeDimensionalVector& v) const {
        return Coord[0] * v.Coord[0] + Coord[1] * v.Coord[1] + Coord[2] * v.Coord[2];
    }

    int64_t Coord[3];
};

struct TwoSet {
    void Insert(int x) { 
        (a == -1 ? a : b) = x; 
    }
    bool Contains(int x) { 
        return a == x || b == x; 
    }
    void Erase(int x) { 
        (a == x ? a : b) = -1; 
    }
    int Size() { 
        return (a != -1) + (b != -1); 
    }
    int a;
    int b;
};

struct ThreeDimensionalFace {
    ThreeDimensionalVector FaceNormal;
    int64_t FaceDiscriminant;
    int FaceVertexIndices[3];
};

ThreeDimensionalFace MakeFace(std::vector<std::vector<TwoSet>>& convex_hull_edges, const std::vector<ThreeDimensionalVector>& points, int i, int j, int k, int inside_i) {
    convex_hull_edges[i][j].Insert(k); 
    convex_hull_edges[i][k].Insert(j); 
    convex_hull_edges[j][k].Insert(i);

    ThreeDimensionalFace f;
    f.FaceVertexIndices[0] = i; 
    f.FaceVertexIndices[1] = j; 
    f.FaceVertexIndices[2] = k;
    f.FaceNormal = (points[j] - points[i]) * (points[k] - points[i]);
    f.FaceDiscriminant = f.FaceNormal.Dot(points[i]);

    if(f.FaceNormal.Dot(points[inside_i]) > f.FaceDiscriminant) {
        f.FaceNormal = -f.FaceNormal;
        f.FaceDiscriminant = -f.FaceDiscriminant;
    }
    return f;
}

void ConvexHull3D(int n, const std::vector<ThreeDimensionalVector>& points, std::vector<ThreeDimensionalFace>& faces) {
    std::vector<std::vector<TwoSet>> convex_hull_edges(n, std::vector<TwoSet>(n));
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            for (int k = j + 1; k < 4; k++) {
                faces.push_back(MakeFace(convex_hull_edges, points, i, j, k, 6 - i - j - k));
            }
        }
    }
    ThreeDimensionalFace f;
    for (int i = 4; i < n; i++) {

        for (size_t j = 0; j < faces.size(); j++) {
            f = faces[j];
            if (f.FaceNormal.Dot(points[i]) > f.FaceDiscriminant) {
                convex_hull_edges[f.FaceVertexIndices[0]][f.FaceVertexIndices[1]].Erase(f.FaceVertexIndices[2]);
                convex_hull_edges[f.FaceVertexIndices[0]][f.FaceVertexIndices[2]].Erase(f.FaceVertexIndices[1]);
                convex_hull_edges[f.FaceVertexIndices[1]][f.FaceVertexIndices[2]].Erase(f.FaceVertexIndices[0]);
                faces[j--] = faces.back();
                faces.resize(faces.size() - 1);
            }
        }

        int n_faces = static_cast<int>(faces.size());
        for (int j = 0; j < n_faces; j++) {
            f = faces[j];
            for (int a = 0; a < 3; a++) {
                for (int b = a + 1; b < 3; b++) {
                    int c = 3 - a - b;
                    if (convex_hull_edges[f.FaceVertexIndices[a]][f.FaceVertexIndices[b]].Size() == 2) {
                        continue;
                    }
                        
                    faces.push_back(MakeFace(convex_hull_edges, points, f.FaceVertexIndices[a], f.FaceVertexIndices[b], i, f.FaceVertexIndices[c]));
                }
            }
        }
    }
}

double CalculateDist(ThreeDimensionalVector v, const std::vector<ThreeDimensionalFace>& faces) {
    double dist = 1e300;
    for (size_t i = 0; i < faces.size(); i++) {
        auto d = static_cast<double>(faces[i].FaceDiscriminant - faces[i].FaceNormal.Dot(v));
        dist = std::min(dist, d / sqrt(static_cast<double>(faces[i].FaceNormal.Dot(faces[i].FaceNormal))));
    }
    return dist;
}

int main() {
    int n = 0;
    std::cin >> n;
    std::vector<ThreeDimensionalVector> points(n);
    for (int i = 0; i < n; ++i) {
        std::cin >> points[i].Coord[0] >> points[i].Coord[1] >> points[i].Coord[2];
    }

    ThreeDimensionalFace f;
    std::vector<ThreeDimensionalFace> faces;
    ConvexHull3D(n, points, faces);

    int q = 0;
    std::cin >> q;
    for (int i = 0; i < q; ++i) {
        ThreeDimensionalVector v;
        std::cin >> v.Coord[0] >> v.Coord[1] >> v.Coord[2];
        std::cout << std::fixed << std::setprecision(4) << CalculateDist(v, faces) << '\n';
    }

    return 0;
}