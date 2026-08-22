#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <deque>
#include <functional>

//g++ -std=c++11 main.cpp -o app   -I/usr/local/include   -L/usr/local/lib   -lglfw   -framework OpenGL   -framework Cocoa   -framework IOKit   -framework CoreVideo -L/Users/macbook2015/Desktop/brew/lib -I/Users/macbook2015/Desktop/brew/include

#define M_PI 3.14159265358979323846f

// ============================================================================
// 1. Math Primitives
// ============================================================================
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = length(); return l > 0 ? *this * (1.0f/l) : Vec3(); }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const { return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x}; }
    float& operator[](int i) { return (&x)[i]; }
    const float& operator[](int i) const { return (&x)[i]; }
};

struct AABB {
    Vec3 min, max;
    AABB() : min(1e30, 1e30, 1e30), max(-1e30, -1e30, -1e30) {}
    AABB(const Vec3& mn, const Vec3& mx) : min(mn), max(mx) {}
    void expand(const Vec3& p) {
        min.x = std::min(min.x, p.x); min.y = std::min(min.y, p.y); min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x); max.y = std::max(max.y, p.y); max.z = std::max(max.z, p.z);
    }
};

struct Triangle {
    Vec3 v0, v1, v2, centroid, normal;
    Triangle() {}
    Triangle(const Vec3& a, const Vec3& b, const Vec3& c) : v0(a), v1(b), v2(c) {
        centroid = (a + b + c) * (1.0f / 3.0f);
    }
};

Vec3 normal(const Vec3& v0, const Vec3& v1, const Vec3& v2) {
    Vec3 a = v1 - v0;
    Vec3 b = v2 - v0;
    return a.cross(b).normalized();
}

float get_noise(const Vec3& dir) {
    float n1 = std::sin(dir.x * 12.9898f + dir.y * 78.233f + dir.z * 45.164f) * 43758.5453f;
    float val1 = n1 - std::floor(n1);
    float n2 = std::sin(dir.x * 45.164f + dir.y * 12.9898f + dir.z * 78.233f) * 22578.1459f;
    float val2 = n2 - std::floor(n2);
    return val1 * 0.7f + val2 * 0.3f;
}

// ============================================================================
// 2. Local Hybrid BSP Tree (Per-Chunk Geometry) - KEPT FOR DEBUG WIREFRAMES
// ============================================================================
class ChunkBSP {
public:
    struct Node {
        AABB bounds;
        uint32_t type; // 0=Quad, 1=BSP, 2=Leaf
        uint32_t c0, c1, c2, c3;
        float splitX, splitY, splitPos;
        int axis;
        uint32_t triStart, triCount;
    };

    void build(const std::vector<Triangle>& tris) {
        nodes.clear(); triIndices.clear();
        if (tris.empty()) return;
        mesh = tris; 
        triIndices.reserve(tris.size());
        for (uint32_t i = 0; i < tris.size(); ++i) triIndices.push_back(i);

        AABB totalBounds;
        for (const auto& t : tris) { totalBounds.expand(t.v0); totalBounds.expand(t.v1); totalBounds.expand(t.v2); }
        nodes.reserve(tris.size() * 2);
        buildRecursive(triIndices, totalBounds, 0);
    }

    const std::vector<Node>& getNodes() const { return nodes; }
    const std::vector<Triangle>& getMesh() const { return mesh; }

private:
    std::vector<Triangle> mesh;
    std::vector<uint32_t> triIndices;
    std::vector<Node> nodes;

    uint32_t buildRecursive(std::vector<uint32_t>& indices, const AABB& bounds, int depth) {
        uint32_t idx = nodes.size();
        nodes.emplace_back();
        nodes[idx].bounds = bounds; 
        nodes[idx].type = 2;

        if (indices.size() <= 8 || depth > 3) { 
            nodes[idx].triStart = triIndices.size(); 
            nodes[idx].triCount = indices.size();
            triIndices.insert(triIndices.end(), indices.begin(), indices.end());
            return idx;
        }

        bool useQuad = depth < 2;
        if (useQuad) {
            nodes[idx].type = 0;
            nodes[idx].splitX = (bounds.min.x + bounds.max.x) * 0.5f;
            nodes[idx].splitY = (bounds.min.y + bounds.max.y) * 0.5f;
            std::vector<uint32_t> q[4];
            for (uint32_t i : indices) {
                const Triangle& t = mesh[i];
                float minX = std::min({t.v0.x, t.v1.x, t.v2.x}), maxX = std::max({t.v0.x, t.v1.x, t.v2.x});
                float minY = std::min({t.v0.y, t.v1.y, t.v2.y}), maxY = std::max({t.v0.y, t.v1.y, t.v2.y});
                if (minX <= nodes[idx].splitX && minY <= nodes[idx].splitY) q[0].push_back(i);
                if (maxX >= nodes[idx].splitX && minY <= nodes[idx].splitY) q[1].push_back(i);
                if (minX <= nodes[idx].splitX && maxY >= nodes[idx].splitY) q[2].push_back(i);
                if (maxX >= nodes[idx].splitX && maxY >= nodes[idx].splitY) q[3].push_back(i);
            }
            AABB b[4] = {bounds, bounds, bounds, bounds};
            b[0].max.x = nodes[idx].splitX; b[0].max.y = nodes[idx].splitY;
            b[1].min.x = nodes[idx].splitX; b[1].max.y = nodes[idx].splitY;
            b[2].max.x = nodes[idx].splitX; b[2].min.y = nodes[idx].splitY;
            b[3].min.x = nodes[idx].splitX; b[3].min.y = nodes[idx].splitY;

            nodes[idx].c0 = buildRecursive(q[0], b[0], depth+1);
            nodes[idx].c1 = buildRecursive(q[1], b[1], depth+1);
            nodes[idx].c2 = buildRecursive(q[2], b[2], depth+1);
            nodes[idx].c3 = buildRecursive(q[3], b[3], depth+1);
        } else {
            nodes[idx].type = 1;
            float ex = bounds.max.x - bounds.min.x, ey = bounds.max.y - bounds.min.y, ez = bounds.max.z - bounds.min.z;
            nodes[idx].axis = (ey > ex && ey > ez) ? 1 : ((ez > ex && ez > ey) ? 2 : 0);
            std::vector<float> cents(indices.size());
            for (size_t i=0; i<indices.size(); ++i) cents[i] = mesh[indices[i]].centroid[nodes[idx].axis];
            std::nth_element(cents.begin(), cents.begin() + cents.size()/2, cents.end());
            nodes[idx].splitPos = cents[cents.size()/2];
            std::vector<uint32_t> left, right;
            int axis = nodes[idx].axis;
            float splitPos = nodes[idx].splitPos;
            for (uint32_t i : indices) {
                const Triangle& t = mesh[i];
                float mn = std::min({t.v0[axis], t.v1[axis], t.v2[axis]});
                float mx = std::max({t.v0[axis], t.v1[axis], t.v2[axis]});
                if (mn <= splitPos) left.push_back(i);
                if (mx >= splitPos) right.push_back(i);
            }
            if (left.size() == indices.size() || right.size() == indices.size()) {
                nodes[idx].type = 2; 
                nodes[idx].triStart = triIndices.size(); 
                nodes[idx].triCount = indices.size();
                triIndices.insert(triIndices.end(), indices.begin(), indices.end());
                return idx;
            }
            AABB lb = bounds, rb = bounds;
            lb.max[axis] = splitPos; rb.min[axis] = splitPos;
            nodes[idx].c0 = buildRecursive(left, lb, depth+1);
            nodes[idx].c1 = buildRecursive(right, rb, depth+1);
        }
        return idx;
    }
};

// ============================================================================
// 3. Logical Entities (Time Machine / Holodeck Programs)
// ============================================================================
struct LogicalEntity {
    enum Type { NONE, HOUSE, ROOM, PAINTING, HUMANOID } type;
    std::string name;
    Vec3 local_pos;
    AABB local_bounds;
    int entropy = 0;
    bool is_backed_up = false;
    Vec3 velocity;
    float wander_timer = 0.0f;
};

// ============================================================================
// 4. The Holodeck Chunk
// ============================================================================
struct HolodeckChunk {
    int cx, cy; 
    ChunkBSP bsp;
    std::vector<Triangle> mesh;
    std::vector<LogicalEntity> entities;
    bool has_tardis = false;
    GLuint dl = 0;

    HolodeckChunk(int chunk_x, int chunk_y, float planet_radius) : cx(chunk_x), cy(chunk_y) {
        generate_geometry(planet_radius);
        check_for_tardis();
        
        // Generate solid lit display list
        dl = glGenLists(1);
        glNewList(dl, GL_COMPILE);
        
        glBegin(GL_TRIANGLES);
        for (const auto& tri : mesh) {
            glNormal3f(tri.normal.x, tri.normal.y, tri.normal.z);
            
            float h0 = (tri.v0.length() - planet_radius) / (planet_radius * 0.05f);
            float h1 = (tri.v1.length() - planet_radius) / (planet_radius * 0.05f);
            float h2 = (tri.v2.length() - planet_radius) / (planet_radius * 0.05f);
            
            auto set_color = [&](float h) {
                if (h < 0.3f) glColor3f(0.1f, 0.3f, 0.7f); // Water
                else if (h < 0.4f) glColor3f(0.76f, 0.70f, 0.50f); // Sand
                else if (h < 0.75f) glColor3f(0.13f, 0.54f, 0.13f); // Grass
                else glColor3f(0.9f, 0.9f, 0.9f); // Snow
            };
            
            set_color(h0); glVertex3f(tri.v0.x, tri.v0.y, tri.v0.z);
            set_color(h1); glVertex3f(tri.v1.x, tri.v1.y, tri.v1.z);
            set_color(h2); glVertex3f(tri.v2.x, tri.v2.y, tri.v2.z);
        }
        glEnd();
        glEndList();
    }
    
    ~HolodeckChunk() {
        if (dl) glDeleteLists(dl, 1);
    }

    void generate_geometry(float radius) {
        float total_chunks_x = 16.0f; 
        float total_chunks_y = 8.0f;
        
        float theta_base = (float(cx % int(total_chunks_x)) / total_chunks_x) * 2.0f * M_PI; 
        float phi_base = (float(cy % int(total_chunks_y)) / total_chunks_y) * M_PI;          

        std::vector<Vec3> verts;
        int res = 16; // Increased resolution
        for (int i = 0; i <= res; ++i) {
            for (int j = 0; j <= res; ++j) {
                float lt = theta_base + (float(j)/res) * (2.0f * M_PI / total_chunks_x);
                float lp = phi_base + (float(i)/res) * (M_PI / total_chunks_y);
                
                Vec3 dir(std::sin(lp) * std::cos(lt), std::cos(lp), std::sin(lp) * std::sin(lt));
                float noise_val = get_noise(dir);
                float disp = noise_val * (radius * 0.05f);
                
                verts.push_back(dir * (radius + disp));
            }
        }

        for (int i = 0; i < res; ++i) {
            for (int j = 0; j < res; ++j) {
                uint32_t a = i * (res + 1) + j;
                uint32_t b = a + res + 1;
                
                Vec3 v0 = verts[a], v1 = verts[b], v2 = verts[a + 1];
                Vec3 v3 = verts[a + 1], v4 = verts[b], v5 = verts[b + 1];
                
                Triangle t1(v0, v1, v2);
                t1.normal = normal(v0, v1, v2);
                mesh.push_back(t1);
                
                Triangle t2(v3, v4, v5);
                t2.normal = normal(v3, v4, v5);
                mesh.push_back(t2);
            }
        }
        bsp.build(mesh);
    }

    void check_for_tardis() {
        if (cx == 8 && cy == 4) {
            has_tardis = true;
            
            Vec3 tardis_pos(-50.0f, 0, 0); // Placed precisely on the surface
            
            LogicalEntity house;
            house.type = LogicalEntity::HOUSE; house.name = "TARDIS House";
            house.local_pos = tardis_pos; 
            house.local_bounds.min = {-1.5f, 0, -1.5f}; house.local_bounds.max = {1.5f, 3, 1.5f};
            entities.push_back(house);

            LogicalEntity room;
            room.type = LogicalEntity::ROOM; room.name = "The Vault";
            room.local_pos = tardis_pos + Vec3(0, 0.1f, 0); 
            room.local_bounds.min = {-1.2f, 0, -1.2f}; room.local_bounds.max = {1.2f, 2.8f, 1.2f};
            entities.push_back(room);

            LogicalEntity painting;
            painting.type = LogicalEntity::PAINTING; painting.name = "Guardian Painting";
            painting.local_pos = tardis_pos + Vec3(0, 1.5f, 1.1f);
            painting.local_bounds.min = {-0.4f, -0.4f, -0.05f}; painting.local_bounds.max = {0.4f, 0.4f, 0.05f};
            entities.push_back(painting);

            LogicalEntity human;
            human.type = LogicalEntity::HUMANOID; human.name = "Wandering Entity";
            human.local_pos = tardis_pos + Vec3(2.0f, 0, 0); 
            human.velocity = {0.8f, 0, 0.8f}; human.wander_timer = 2.0f;
            human.local_bounds.min = {-0.2f, 0, -0.2f}; human.local_bounds.max = {0.2f, 1.0f, 0.2f};
            entities.push_back(human);
        }
    }

    void update_ai(float dt) {
        for (auto& e : entities) {
            if (e.type == LogicalEntity::HUMANOID || e.type == LogicalEntity::PAINTING) {
                e.wander_timer -= dt;
                if (e.wander_timer <= 0) {
                    e.velocity = Vec3((float(rand()%200)/100.0f)-1.0f, 0, (float(rand()%200)/100.0f)-1.0f) * 1.5f;
                    e.wander_timer = 1.0f + float(rand()%200)/100.0f;
                }
                e.local_pos = e.local_pos + e.velocity * dt;
                
                // Keep human on surface
                if (e.type == LogicalEntity::HUMANOID) {
                    Vec3 dir = e.local_pos.normalized();
                    float h = get_noise(dir);
                    float r = 50.0f + h * (50.0f * 0.05f);
                    e.local_pos = dir * (r + 0.0f);
                    
                    // Keep human near the house
                    float dist_to_house = (e.local_pos - Vec3(-50.0f, 0, 0)).length();
                    if (dist_to_house > 5.0f) {
                        e.velocity = (Vec3(-50.0f, 0, 0) - e.local_pos).normalized() * 1.5f;
                    }
                }
            }
        }
    }
};

// ============================================================================
// 5. The Sliding Holodeck Grid
// ============================================================================
class PlanetaryHolodeck {
public:
    int width, height;
    std::deque<std::deque<HolodeckChunk*>> grid;
    int origin_x, origin_y;
    float planet_radius;

    PlanetaryHolodeck(int w, int h, float r) : width(w), height(h), origin_x(0), origin_y(0), planet_radius(r) {
        grid.resize(height);
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                grid[i].push_back(new HolodeckChunk(origin_x + j, origin_y + i, r));
            }
        }
    }

    ~PlanetaryHolodeck() {
        for (auto& row : grid) for (auto* chunk : row) delete chunk;
    }

    void shift_x(int dir) {
        if (dir > 0) { 
            for (int i = 0; i < height; ++i) {
                delete grid[i].front(); grid[i].pop_front();
                grid[i].push_back(new HolodeckChunk(origin_x + width, origin_y + i, planet_radius));
            }
            origin_x++;
        } else {
            for (int i = 0; i < height; ++i) {
                delete grid[i].back(); grid[i].pop_back();
                grid[i].push_front(new HolodeckChunk(origin_x - 1, origin_y + i, planet_radius));
            }
            origin_x--;
        }
    }

    void shift_y(int dir) {
        if (dir > 0) { 
            for (auto* chunk : grid.front()) delete chunk; grid.pop_front();
            std::deque<HolodeckChunk*> new_row;
            for (int j = 0; j < width; ++j) new_row.push_back(new HolodeckChunk(origin_x + j, origin_y + height, planet_radius));
            grid.push_back(new_row);
            origin_y++;
        } else {
            for (auto* chunk : grid.back()) delete chunk; grid.pop_back();
            std::deque<HolodeckChunk*> new_row;
            for (int j = 0; j < width; ++j) new_row.push_front(new HolodeckChunk(origin_x + j, origin_y - 1, planet_radius));
            grid.push_front(new_row);
            origin_y--;
        }
    }

    void update_ai(float dt) {
        for (auto& row : grid) for (auto* chunk : row) chunk->update_ai(dt);
    }
};

// ============================================================================
// 6. Time Machine Logic
// ============================================================================
void backup_tardis(PlanetaryHolodeck& holo) {
    for (auto& row : holo.grid) for (auto* chunk : row) if (chunk->has_tardis) for (auto& e : chunk->entities)
        if (e.type == LogicalEntity::ROOM) { e.is_backed_up = true; e.entropy = 0; std::cout << "\n[GUARDIAN PAINTING]: 'I have memorized this room''s exact state.'\n"; }
}

void advance_time(PlanetaryHolodeck& holo) {
    for (auto& row : holo.grid) for (auto* chunk : row) if (chunk->has_tardis) for (auto& e : chunk->entities)
        if (e.type == LogicalEntity::ROOM) { e.entropy += 15; std::cout << "[GUARDIAN PAINTING]: 'Entropy is now " << e.entropy << "'\n"; }
}

void restore_tardis(PlanetaryHolodeck& holo) {
    for (auto& row : holo.grid) for (auto* chunk : row) if (chunk->has_tardis) for (auto& e : chunk->entities) {
        if (e.type == LogicalEntity::ROOM && e.is_backed_up) { e.entropy = 0; std::cout << "[GUARDIAN PAINTING]: 'The timeline is rewritten!'\n"; }
        if (e.type == LogicalEntity::HUMANOID) e.local_pos = Vec3(-50.0f, 0, 0) + Vec3(2.0f, 0, 0); 
    }
}

// ============================================================================
// 7. OpenGL Rendering Helpers
// ============================================================================
void drawSolidBox(const AABB& bounds, Vec3 color) {
    glColor3f(color.x, color.y, color.z);
    glBegin(GL_QUADS);
    Vec3 min = bounds.min, max = bounds.max;
    glNormal3f(0, 0, 1); glVertex3f(min.x, min.y, max.z); glVertex3f(max.x, min.y, max.z); glVertex3f(max.x, max.y, max.z); glVertex3f(min.x, max.y, max.z);
    glNormal3f(0, 0, -1); glVertex3f(max.x, min.y, min.z); glVertex3f(min.x, min.y, min.z); glVertex3f(min.x, max.y, min.z); glVertex3f(max.x, max.y, min.z);
    glNormal3f(0, 1, 0); glVertex3f(min.x, max.y, max.z); glVertex3f(max.x, max.y, max.z); glVertex3f(max.x, max.y, min.z); glVertex3f(min.x, max.y, min.z);
    glNormal3f(0, -1, 0); glVertex3f(min.x, min.y, min.z); glVertex3f(max.x, min.y, min.z); glVertex3f(max.x, min.y, max.z); glVertex3f(min.x, min.y, max.z);
    glNormal3f(1, 0, 0); glVertex3f(max.x, min.y, max.z); glVertex3f(max.x, min.y, min.z); glVertex3f(max.x, max.y, min.z); glVertex3f(max.x, max.y, max.z);
    glNormal3f(-1, 0, 0); glVertex3f(min.x, min.y, min.z); glVertex3f(min.x, min.y, max.z); glVertex3f(min.x, max.y, max.z); glVertex3f(min.x, max.y, min.z);
    glEnd();
}

void drawHouse() {
    drawSolidBox({{-1.5f, 0.0f, -1.5f}, {1.5f, 3.0f, 1.5f}}, {0.6f, 0.4f, 0.2f}); // Main Body
    drawSolidBox({{-1.8f, 3.0f, -1.8f}, {1.8f, 3.5f, 1.8f}}, {0.5f, 0.1f, 0.1f}); // Roof
    drawSolidBox({{-0.5f, 0.0f, 1.5f}, {0.5f, 2.0f, 1.55f}}, {0.3f, 0.2f, 0.1f}); // Door
}

void drawWireBox(const AABB& bounds, Vec3 offset, float r, float g, float b, float lw = 1.0f) {
    glLineWidth(lw); glColor3f(r, g, b); glBegin(GL_LINES);
    Vec3 min = bounds.min + offset, max = bounds.max + offset;
    glVertex3f(min.x, min.y, min.z); glVertex3f(max.x, min.y, min.z); glVertex3f(max.x, min.y, min.z); glVertex3f(max.x, min.y, max.z);
    glVertex3f(max.x, min.y, max.z); glVertex3f(min.x, min.y, max.z); glVertex3f(min.x, min.y, max.z); glVertex3f(min.x, min.y, min.z);
    glVertex3f(min.x, max.y, min.z); glVertex3f(max.x, max.y, min.z); glVertex3f(max.x, max.y, min.z); glVertex3f(max.x, max.y, max.z);
    glVertex3f(max.x, max.y, max.z); glVertex3f(min.x, max.y, max.z); glVertex3f(min.x, max.y, max.z); glVertex3f(min.x, max.y, min.z);
    glVertex3f(min.x, min.y, min.z); glVertex3f(min.x, max.y, min.z); glVertex3f(max.x, min.y, min.z); glVertex3f(max.x, max.y, min.z);
    glVertex3f(max.x, min.y, max.z); glVertex3f(max.x, max.y, max.z); glVertex3f(min.x, min.y, max.z); glVertex3f(min.x, max.y, max.z);
    glEnd();
}

void draw_chunk_bsp(const ChunkBSP& bsp, uint32_t idx, int depth, int maxDepth, Vec3 offset) {
    if (idx >= bsp.getNodes().size() || depth > maxDepth) return;
    const auto& n = bsp.getNodes()[idx];
    if (n.type == 0) glColor3f(0.0f, 1.0f, 1.0f); else if (n.type == 1) glColor3f(0.0f, 1.0f, 0.0f); else return;
    drawWireBox(n.bounds, offset, 0,0,0, 1.0f);
    if (n.type == 0) { draw_chunk_bsp(bsp, n.c0, depth+1, maxDepth, offset); draw_chunk_bsp(bsp, n.c1, depth+1, maxDepth, offset); draw_chunk_bsp(bsp, n.c2, depth+1, maxDepth, offset); draw_chunk_bsp(bsp, n.c3, depth+1, maxDepth, offset); } 
    else { draw_chunk_bsp(bsp, n.c0, depth+1, maxDepth, offset); draw_chunk_bsp(bsp, n.c1, depth+1, maxDepth, offset); }
}

// ============================================================================
// 8. Main Application
// ============================================================================
int main() {
    if (!glfwInit()) return -1;
    GLFWwindow* window = glfwCreateWindow(1280, 720, "Planetary Holodeck + Time Machine", NULL, NULL);
    if (!window) { glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    std::cout << "Initializing Planetary Holodeck...\n";
    float planet_radius = 50.0f;
    PlanetaryHolodeck holodeck(5, 5, planet_radius); 

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glShadeModel(GL_SMOOTH);

    float ambient[] = {0.4f, 0.4f, 0.4f, 1.0f};
    float diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
    float spec[] = {0.5f, 0.5f, 0.5f, 1.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, spec);

    Vec3 camPos(0, 0, planet_radius + 1.5f);
    float camYaw = 0.0f;
    float camPitch = 0.0f;
    bool orbit_mode = false;
    float orbit_dist = planet_radius * 3.0f;
    double lastX = 640.0, lastY = 360.0;
    bool firstMouse = true;
    int bsp_depth = 0;

    int last_grid_cx = 4, last_grid_cy = 4; // Match initial camPos
    float unwrapped_theta = M_PI / 2.0f;

    std::cout << "\n--- HOLODECK CONTROLS ---\n";
    std::cout << "Mouse: Look around (FPS)\n";
    std::cout << "W/A/S/D: Walk on Planet Surface (Triggers Grid Sliding!)\n";
    std::cout << "Z: Toggle Orbit Mode (Zoom out to see planet)\n";
    std::cout << "PageUp/PageDown: Zoom In/Out (Orbit Mode)\n";
    std::cout << "Up/Down Arrows: Adjust Chunk BSP Wireframe Depth\n";
    std::cout << "T: Advance Time | B: BACKUP | R: RESTORE\n";
    std::cout << "ESC: Quit\n\n";

    auto lastTime = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window)) {
        auto now = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;

        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(window, true);

        static int frameCount = 0;
        if (frameCount++ % 10 == 0) {
            if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS && bsp_depth < 5) bsp_depth++;
            if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS && bsp_depth > 0) bsp_depth--;
        }

        float moveSpeed = 15.0f * dt;
        if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
            orbit_mode = !orbit_mode;
            if (orbit_mode) glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            else glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            while(glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) glfwPollEvents();
        }
        
        if (glfwGetKey(window, GLFW_KEY_PAGE_UP) == GLFW_PRESS) orbit_dist -= 50.0f * dt;
        if (glfwGetKey(window, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS) orbit_dist += 50.0f * dt;
        if (orbit_dist < planet_radius * 1.2f) orbit_dist = planet_radius * 1.2f;

        Vec3 N = camPos.normalized();
        Vec3 ref_up(0, 1, 0);
        if (std::abs(N.y) > 0.99f) ref_up = Vec3(0, 0, 1);
        Vec3 T = ref_up.cross(N).normalized(); 
        Vec3 B = N.cross(T); 

        Vec3 local_forward = B * std::cos(camYaw) + T * std::sin(camYaw);
        Vec3 local_right = T * std::cos(camYaw) - B * std::sin(camYaw);

        if (!orbit_mode) {
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) camPos = camPos + local_forward * moveSpeed;
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) camPos = camPos - local_forward * moveSpeed;
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) camPos = camPos - local_right * moveSpeed;
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) camPos = camPos + local_right * moveSpeed;
            
            N = camPos.normalized();
            float noise_val = get_noise(N);
            float terrain_radius = planet_radius + noise_val * (planet_radius * 0.05f);
            camPos = N * (terrain_radius + 1.5f); // Clamp to surface + player height
        } else {
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) camPitch += 1.5f * dt;
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) camPitch -= 1.5f * dt;
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) camYaw -= 1.5f * dt;
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) camYaw += 1.5f * dt;
            
            if (camPitch > 1.5f) camPitch = 1.5f;
            if (camPitch < -1.5f) camPitch = -1.5f;
            
            camPos = Vec3(orbit_dist * std::cos(camPitch) * std::sin(camYaw), 
                          orbit_dist * std::sin(camPitch), 
                          orbit_dist * std::cos(camPitch) * std::cos(camYaw));
        }

        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        if (firstMouse) { lastX = xpos; lastY = ypos; firstMouse = false; }
        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos; 
        lastX = xpos; lastY = ypos;

        camYaw += xoffset * 0.005f;
        camPitch += yoffset * 0.005f;
        if (camPitch > 1.5f) camPitch = 1.5f;
        if (camPitch < -1.5f) camPitch = -1.5f;

        // Shift grid logic
        N = camPos.normalized();
        float theta = std::atan2(N.z, N.x);
        if (theta < 0) theta += 2.0f * M_PI;
        float delta = theta - std::fmod(unwrapped_theta, 2.0f * M_PI);
        if (delta > M_PI) delta -= 2.0f * M_PI;
        if (delta < -M_PI) delta += 2.0f * M_PI;
        unwrapped_theta += delta;

        int current_cx = int((unwrapped_theta / (2.0f * M_PI)) * 16.0f);
        float phi = std::acos(std::max(-1.0f, std::min(1.0f, N.y)));
        int current_cy = int((phi / M_PI) * 8.0f);

        if (current_cx != last_grid_cx) {
            int diff = current_cx - last_grid_cx;
            int dir = diff > 0 ? 1 : -1;
            while (diff != 0) { holodeck.shift_x(dir); diff -= dir; }
            last_grid_cx = current_cx;
            std::cout << "[HOLODECK] Streaming new chunk column...\n";
        }
        if (current_cy != last_grid_cy) {
            int diff = current_cy - last_grid_cy;
            int dir = diff > 0 ? 1 : -1;
            while (diff != 0) { holodeck.shift_y(dir); diff -= dir; }
            last_grid_cy = current_cy;
            std::cout << "[HOLODECK] Streaming new chunk row...\n";
        }

        static bool t_pressed = false, b_pressed = false, r_pressed = false;
        if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS && !t_pressed) { t_pressed = true; advance_time(holodeck); }
        if (glfwGetKey(window, GLFW_KEY_T) == GLFW_RELEASE) t_pressed = false;
        if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS && !b_pressed) { b_pressed = true; backup_tardis(holodeck); }
        if (glfwGetKey(window, GLFW_KEY_B) == GLFW_RELEASE) b_pressed = false;
        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS && !r_pressed) { r_pressed = true; restore_tardis(holodeck); }
        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_RELEASE) r_pressed = false;

        holodeck.update_ai(dt);

        float light_pos[] = { camPos.x + 100.0f, camPos.y + 100.0f, camPos.z + 100.0f, 1.0f };
        glLightfv(GL_LIGHT0, GL_POSITION, light_pos);

        glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION); glLoadIdentity();
        float aspect = 1280.0f / 720.0f;
        float fov = 1.0f / std::tan(45.0f * M_PI / 360.0f);
        float proj[16] = {fov/aspect,0,0,0, 0,fov,0,0, 0,0,-1.002f,-1, 0,0,-0.2f,0};
        glLoadMatrixf(proj);

        glMatrixMode(GL_MODELVIEW); glLoadIdentity();

        if (orbit_mode) {
            Vec3 eye = camPos;
            Vec3 center(0, 0, 0);
            Vec3 up(0, 1, 0);
            Vec3 f = (center - eye).normalized();
            Vec3 s = f.cross(up).normalized();
            Vec3 u = s.cross(f);
            float m[16] = { s.x, u.x, -f.x, 0.0f, s.y, u.y, -f.y, 0.0f, s.z, u.z, -f.z, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
            glMultMatrixf(m);
            glTranslatef(-eye.x, -eye.y, -eye.z);
        } else {
            Vec3 eye = camPos;
            N = eye.normalized();
            ref_up = Vec3(0, 1, 0);
            if (std::abs(N.y) > 0.99f) ref_up = Vec3(0, 0, 1);
            T = ref_up.cross(N).normalized(); 
            B = N.cross(T); 
            local_forward = B * std::cos(camYaw) + T * std::sin(camYaw);
            Vec3 view_forward = local_forward * std::cos(camPitch) + N * std::sin(camPitch);
            Vec3 view_up = N * std::cos(camPitch) - local_forward * std::sin(camPitch);
            
            Vec3 center = eye + view_forward;
            Vec3 f = (center - eye).normalized();
            Vec3 s = f.cross(view_up).normalized();
            Vec3 u = s.cross(f);
            float m[16] = { s.x, u.x, -f.x, 0.0f, s.y, u.y, -f.y, 0.0f, s.z, u.z, -f.z, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
            glMultMatrixf(m);
            glTranslatef(-eye.x, -eye.y, -eye.z);
        }

        for (int i = 0; i < holodeck.height; ++i) {
            for (int j = 0; j < holodeck.width; ++j) {
                HolodeckChunk* chunk = holodeck.grid[i][j];
                glCallList(chunk->dl);
                
                if (bsp_depth > 0) {
                    glDisable(GL_LIGHTING);
                    draw_chunk_bsp(chunk->bsp, 0, 0, bsp_depth, {0,0,0});
                    glEnable(GL_LIGHTING);
                }

                if (chunk->has_tardis) {
                    for (const auto& e : chunk->entities) {
                        glPushMatrix();
                        glTranslatef(e.local_pos.x, e.local_pos.y, e.local_pos.z);
                        
                        Vec3 up = e.local_pos.normalized();
                        Vec3 r_up(0, 1, 0);
                        if (std::abs(up.y) > 0.99f) r_up = Vec3(0, 0, 1);
                        Vec3 right = r_up.cross(up).normalized();
                        Vec3 forward = up.cross(right);

                        float rotMat[16] = {
                            right.x, up.x, forward.x, 0.0f,
                            right.y, up.y, forward.y, 0.0f,
                            right.z, up.z, forward.z, 0.0f,
                            0.0f, 0.0f, 0.0f, 1.0f
                        };
                        glMultMatrixf(rotMat);

                        if (e.type == LogicalEntity::HOUSE) drawHouse();
                        else if (e.type == LogicalEntity::ROOM) drawSolidBox(e.local_bounds, {0.3f, 0.3f, 0.5f});
                        else if (e.type == LogicalEntity::PAINTING) drawSolidBox(e.local_bounds, {0.8f, 0.8f, 0.2f});
                        else if (e.type == LogicalEntity::HUMANOID) drawSolidBox(e.local_bounds, {0.8f, 0.6f, 0.5f});

                        glPopMatrix();
                    }
                }
            }
        }

        glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(-1, 1, -1, 1, -1, 1);
        glMatrixMode(GL_MODELVIEW); glLoadIdentity(); 
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_LINES);
        glVertex2f(-0.02f, 0.0f); glVertex2f(0.02f, 0.0f);
        glVertex2f(0.0f, -0.02f); glVertex2f(0.0f, 0.02f);
        glEnd();
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}