#include "resource/mesh.h"

#include <fstream>
#include <sstream>
#include <string>

const float kFloatLimit = 1e-5f;

Mesh::Mesh(const std::string& filename) {
    std::ifstream in;
    in.open(filename, std::ios::in);
    if (in.fail()) return;
    std::string line;
    bound_ = std::make_pair<Vec3f, Vec3f>(
        Vec3f{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()},
        Vec3f{std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min()});
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line);
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vec3f v;
            for (size_t i = 0; i < 3; ++i) {
                iss >> v[i];
                bound_.first[i] = std::min(bound_.first[i], v[i]);
                bound_.second[i] = std::max(bound_.second[i], v[i]);
            }
            verts_.emplace_back(v);
        } else if (!line.compare(0, 2, "f ")) {
            std::vector<Vertex> face;
            int idx, uv_idx, normal_idx;
            iss >> trash;
            if (line.find("/") != std::string::npos) {
                while (iss >> idx >> trash >> uv_idx >> trash >> normal_idx) {
                    --idx;  // start from 1 not 0
                    --uv_idx;
                    --normal_idx;
                    face.emplace_back(idx, uv_idx, normal_idx);
                }
            } else {
                while (iss >> idx) {
                    --idx;
                    face.emplace_back(idx, -1, -1);
                }
            }
            faces_.push_back(std::move(face));
        } else if (!line.compare(0, 2, "vt")) {
            iss >> trash >> trash;
            Vec3f v;
            for (size_t i = 0; i < 3; ++i) {
                iss >> v[i];
            }
            uvs_.emplace_back(v);
        } else if (!line.compare(0, 2, "vn")) {
            iss >> trash >> trash;
            Vec3f v;
            for (size_t i = 0; i < 3; ++i) {
                iss >> v[i];
            }
            normals_.emplace_back(v);
        }
    }
}

Mesh::~Mesh() = default;

size_t Mesh::nverts() const { return verts_.size(); }

size_t Mesh::nfaces() const { return faces_.size(); }

Vec3f Mesh::vert(const int& idx) const { return verts_[idx]; }

Vec3f Mesh::normal(const int& idx) const { return normals_[idx]; }
Vec2f Mesh::uv(const int& idx) const { return projection<2>(uvs_[idx]); }

std::vector<int> Mesh::face_vertex(const int& idx) const {
    std::vector<int> ans;
    for (const auto& vertex : face(idx)) {
        ans.emplace_back(vertex.vertIdx());
    }
    return ans;
}
std::vector<int> Mesh::face_uv(const int& idx) const {
    std::vector<int> ans;
    for (const auto& vertex : face(idx)) {
        ans.emplace_back(vertex.uvIdx());
    }
    return ans;
}
std::vector<int> Mesh::face_normal(const int& idx) const {
    std::vector<int> ans;
    for (const auto& vertex : face(idx)) {
        ans.emplace_back(vertex.normalIdx());
    }
    return ans;
}

const std::vector<Mesh::Vertex>& Mesh::face(const int& idx) const { return faces_[idx]; }

bool Mesh::ray_triangle_intersect(const int& faceIndex, const Vec3f& orig, const Vec3f& dir, float& tnear, float& u,
                                  float& v) const {
    const auto& face_ = face(faceIndex);
    Vec3f edge1 = vert(face_[1].vertIdx()) - vert(face_[0].vertIdx());
    Vec3f edge2 = vert(face_[2].vertIdx()) - vert(face_[0].vertIdx());
    Vec3f pvec = cross(dir, edge2);
    float det = edge1 * pvec;
    if (det < kFloatLimit) return false;

    Vec3f tvec = orig - vert(face_[0].vertIdx());
    v = tvec * pvec;
    if (v < 0 || v > det) return false;

    Vec3f qvec = cross(tvec, edge1);
    u = dir * qvec;
    if (u < 0 || u + v > det) return false;

    tnear = edge2 * qvec * (1.f / det);
    return tnear > kFloatLimit;
}
