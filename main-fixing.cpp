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

// ============================================================================
// 2. Ken Perlin's Improved Noise
// ============================================================================
static const int perm[512] = {
    151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,
    247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,
    74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,
    54,65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,200,196,135,130,116,188,159,86,164,100,109,198,173,186,
    3,64,52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,223,183,
    170,213,119,248,152,2,44,154,163,70,221,153,101,155,167,43,172,9,129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,
    218,246,97,228,251,34,242,193,238,210,144,12,191,179,162,241,81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,184,
    84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180,
    151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,
    247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,
    74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,
    54,65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,200,196,135,130,116,188,159,86,164,100,109,198,173,186,
    3,64,52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,223,183,
    170,213,119,248,152,2,44,154,163,70,221,153,101,155,167,43,172,9,129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,
    218,246,97,228,251,34,242,193,238,210,144,12,191,179,162,241,81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,184,
    84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
};

inline float fade(float t) { return t * t * t * (t * (t * 6 - 15) + 10); }
inline float lerp(float t, float a, float b) { return a + t * (b - a); }
inline float grad(int hash, float x, float y, float z) {
    int h = hash & 15;
    float u = h < 8 ? x : y, v = h < 4 ? y : h == 12 || h == 14 ? x : z;
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

float perlin_noise(float x, float y, float z) {
    int X = (int)std::floor(x) & 255;
    int Y = (int)std::floor(y) & 255;
    int Z = (int)std::floor(z) & 255;
    x -= std::floor(x); y -= std::floor(y); z -= std::floor(z);
    float u = fade(x), v = fade(y), w = fade(z);
    int A = perm[X] + Y, AA = perm[A] + Z, AB = perm[A + 1] + Z;
    int B = perm[X + 1] + Y, BA = perm[B] + Z, BB = perm[B + 1] + Z;
    return lerp(w, lerp(v, lerp(u, grad(perm[AA], x, y, z), grad(perm[BA], x - 1, y, z)),
                           lerp(u, grad(perm[AB], x, y - 1, z), grad(perm[BB], x - 1, y - 1, z))),
                   lerp(v, lerp(u, grad(perm[AA + 1], x, y, z - 1), grad(perm[BA + 1], x - 1, y, z - 1)),
                           lerp(u, grad(perm[AB + 1], x, y - 1, z - 1), grad(perm[BB + 1], x - 1, y - 1, z - 1))));
}

float get_noise(const Vec3& dir) {
    float scale = 2.5f; 
    Vec3 p = dir * scale;
    float val = 0.0f, amp = 0.55f, freq = 1.0f, max_amp = 0.0f;
    for(int i=0; i<5; ++i) {
        val += amp * perlin_noise(p.x * freq, p.y * freq, p.z * freq);
        freq *= 2.1f; amp *= 0.48f; max_amp += amp;
    }
    float normalized = (val / max_amp + 1.0f) * 0.5f;
    return std::max(0.0f, std::min(1.0f, normalized));
}

// ============================================================================
// 3. Logical Entities
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
    std::vector<LogicalEntity> entities;
    bool has_tardis = false;
    GLuint dl = 0;

    HolodeckChunk(int chunk_x, int chunk_y, float planet_radius) : cx(chunk_x), cy(chunk_y) {
        generate_geometry(planet_radius);
        check_for_tardis();
    }
    
    ~HolodeckChunk() { if (dl) glDeleteLists(dl, 1); }

    struct Vertex { Vec3 pos, normal; };

    void generate_geometry(float radius) {
        float total_chunks_x = 16.0f; 
        float total_chunks_y = 8.0f;
        
        float wrapped_cx = std::fmod(float(cx), total_chunks_x);
        if (wrapped_cx < 0) wrapped_cx += total_chunks_x;
        
        float theta_base = (wrapped_cx / total_chunks_x) * 2.0f * M_PI; 
        float phi_base = (float(cy) / total_chunks_y) * M_PI;          
        float phi_end = (float(cy + 1) / total_chunks_y) * M_PI;

        float cos_phi_start = std::cos(phi_base);
        float cos_phi_end = std::cos(phi_end);

        std::vector<Vertex> verts;
        int res = 40; 
        verts.reserve((res + 1) * (res + 1));
        
        for (int i = 0; i <= res; ++i) {
            for (int j = 0; j <= res; ++j) {
                float u = float(j) / res;
                float v = float(i) / res;
                
                float lt = theta_base + u * (2.0f * M_PI / total_chunks_x);
                float current_cos_phi = cos_phi_start + v * (cos_phi_end - cos_phi_start);
                current_cos_phi = std::max(-1.0f, std::min(1.0f, current_cos_phi));
                float lp = std::acos(current_cos_phi);
                
                Vec3 dir(std::sin(lp) * std::cos(lt), std::cos(lp), std::sin(lp) * std::sin(lt));
                float noise_val = get_noise(dir);
                float disp = noise_val * (radius * 0.05f);
                
                verts.push_back({dir * (radius + disp), Vec3(0,0,0)});
            }
        }

        struct Tri { int i0, i1, i2; };
        std::vector<Tri> tris;
        tris.reserve(res * res * 2);

        for (int i = 0; i < res; ++i) {
            for (int j = 0; j < res; ++j) {
                int a = i * (res + 1) + j;
                int b = a + res + 1;
                int c = a + 1;
                int d = b + 1;
                tris.push_back({a, b, c});
                tris.push_back({c, b, d});
            }
        }

        for (const auto& t : tris) {
            Vec3 n = normal(verts[t.i0].pos, verts[t.i1].pos, verts[t.i2].pos);
            verts[t.i0].normal = verts[t.i0].normal + n;
            verts[t.i1].normal = verts[t.i1].normal + n;
            verts[t.i2].normal = verts[t.i2].normal + n;
        }

        for (auto& v : verts) v.normal = v.normal.normalized();

        dl = glGenLists(1);
        glNewList(dl, GL_COMPILE);
        glShadeModel(GL_SMOOTH);
        
        glBegin(GL_TRIANGLES);
        for (const auto& t : tris) {
            auto draw_v = [&](int idx) {
                const auto& v = verts[idx];
                glNormal3f(v.normal.x, v.normal.y, v.normal.z);
                float h = std::max(0.0f, std::min(1.0f, (v.pos.length() - radius) / (radius * 0.05f)));
                if (h < 0.35f) glColor3f(0.1f, 0.3f, 0.7f); 
                else if (h < 0.4f) glColor3f(0.2f, 0.4f, 0.8f); 
                else if (h < 0.45f) glColor3f(0.76f, 0.70f, 0.50f); 
                else if (h < 0.75f) glColor3f(0.13f, 0.54f, 0.13f); 
                else glColor3f(0.9f, 0.9f, 0.95f); 
                glVertex3f(v.pos.x, v.pos.y, v.pos.z);
            };
            draw_v(t.i0); draw_v(t.i1); draw_v(t.i2);
        }
        glEnd();
        glEndList();
    }

    void check_for_tardis() {
        float wrapped_cx = std::fmod(float(cx), 16.0f);
        if (wrapped_cx < 0) wrapped_cx += 16.0f;
        
        if (int(wrapped_cx) == 8 && cy == 4) {
            has_tardis = true;
            
            float total_chunks_x = 16.0f, total_chunks_y = 8.0f;
            float center_theta = (wrapped_cx + 0.5f) / total_chunks_x * 2.0f * M_PI;
            float phi_base = (float(cy) / total_chunks_y) * M_PI;
            float phi_end = (float(cy + 1) / total_chunks_y) * M_PI;
            float cos_center = std::cos(phi_base) + 0.5f * (std::cos(phi_end) - std::cos(phi_base));
            float center_phi = std::acos(cos_center);
            
            Vec3 tardis_dir(std::sin(center_phi) * std::cos(center_theta), std::cos(center_phi), std::sin(center_phi) * std::sin(center_theta));
            tardis_dir = tardis_dir.normalized();
            
            float noise_val = get_noise(tardis_dir);
            float r = 50.0f + noise_val * (50.0f * 0.05f);
            Vec3 tardis_pos = tardis_dir * r;
            
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
            Vec3 up = tardis_pos.normalized();
            Vec3 r_up(0, 1, 0);
            if (std::abs(up.y) > 0.99f) r_up = Vec3(0, 0, 1);
            Vec3 right = up.cross(r_up).normalized(); // Fixed right-handed cross product
            human.local_pos = (tardis_pos + right * 2.0f).normalized() * tardis_pos.length();
            human.velocity = {0.8f, 0, 0.8f}; human.wander_timer = 2.0f;
            human.local_bounds.min = {-0.2f, 0, -0.2f}; human.local_bounds.max = {0.2f, 1.0f, 0.2f};
            entities.push_back(human);
        }
    }

    void update_ai(float dt) {
        for (auto& e : entities) {
            if (e.type == LogicalEntity::HUMANOID) {
                e.wander_timer -= dt;
                if (e.wander_timer <= 0) {
                    e.velocity = Vec3((float(rand()%200)/100.0f)-1.0f, 0, (float(rand()%200)/100.0f)-1.0f) * 1.5f;
                    e.wander_timer = 1.0f + float(rand()%200)/100.0f;
                }
                e.local_pos = e.local_pos + e.velocity * dt;
                
                Vec3 dir = e.local_pos.normalized();
                float h = get_noise(dir);
                float r = 50.0f + h * (50.0f * 0.05f);
                e.local_pos = dir * (r + 0.0f);
                
                Vec3 tardis_pos = entities[0].local_pos;
                float dist_to_house = (e.local_pos - tardis_pos).length();
                if (dist_to_house > 5.0f) e.velocity = (tardis_pos - e.local_pos).normalized() * 1.5f;
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
        int start_cx = 8 - w/2;
        int start_cy = 4 - h/2;
        origin_x = start_cx;
        origin_y = start_cy;
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
            for (int j = 0; j < width; ++j) new_row.push_back(new HolodeckChunk(origin_x + j, origin_y - 1, planet_radius));
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
        if (e.type == LogicalEntity::HUMANOID) {
            Vec3 tardis_pos = chunk->entities[0].local_pos;
            Vec3 up = tardis_pos.normalized();
            Vec3 r_up(0, 1, 0);
            if (std::abs(up.y) > 0.99f) r_up = Vec3(0, 0, 1);
            Vec3 right = up.cross(r_up).normalized(); // Fixed right-handed cross product
            e.local_pos = (tardis_pos + right * 2.0f).normalized() * tardis_pos.length(); 
        }
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
    drawSolidBox({{-1.5f, 0.0f, -1.5f}, {1.5f, 3.0f, 1.5f}}, {0.6f, 0.4f, 0.2f}); 
    drawSolidBox({{-1.8f, 3.0f, -1.8f}, {1.8f, 3.5f, 1.8f}}, {0.5f, 0.1f, 0.1f}); 
    drawSolidBox({{-0.5f, 0.0f, 1.5f}, {0.5f, 2.0f, 1.55f}}, {0.3f, 0.2f, 0.1f}); 
}

bool was_at_pole;
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

    std::cout << "Initializing Sliding Planetary Holodeck...\n";
    float planet_radius = 50.0f;
    
    PlanetaryHolodeck holodeck(7, 5, planet_radius); 

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

    float last_theta = std::atan2(camPos.z, camPos.x);
    if (last_theta < 0) last_theta += 2.0f * M_PI;
    float unwrapped_theta = last_theta;

    std::cout << "\n--- HOLODECK CONTROLS ---\n";
    std::cout << "Mouse: Look around (FPS)\n";
    std::cout << "W/A/S/D: Walk on Planet Surface (Triggers Streaming)\n";
    std::cout << "Z: Toggle Orbit Mode (Zoom out to see planet)\n";
    std::cout << "PageUp/PageDown: Zoom In/Out (Orbit Mode)\n";
    std::cout << "T: Advance Time | B: BACKUP | R: RESTORE\n";
    std::cout << "ESC: Quit\n\n";

    auto lastTime = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window)) {
auto now = std::chrono::high_resolution_clock::now();
float dt = std::chrono::duration<float>(now - lastTime).count();
lastTime = now;

// CRITICAL FIX 1: Clamp dt to prevent "death spiral" lag spikes 
// from causing massive movement jumps and chunk reloads
if (dt > 0.05f) dt = 0.05f; 

if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(window, true);

if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
    orbit_mode = !orbit_mode;
    if (orbit_mode) glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    else glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    while(glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) glfwPollEvents();
}

if (glfwGetKey(window, GLFW_KEY_PAGE_UP) == GLFW_PRESS) orbit_dist -= 150.0f * dt;
if (glfwGetKey(window, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS) orbit_dist += 150.0f * dt;
if (orbit_dist < planet_radius * 1.2f) orbit_dist = planet_radius * 1.2f;
if (orbit_dist > 5000.0f) orbit_dist = 5000.0f;

// CRITICAL FIX 2: Calculate vectors safely
Vec3 N = camPos.normalized();
Vec3 ref_up(0, 1, 0);
if (std::abs(N.y) > 0.98f) ref_up = Vec3(0, 0, 1); // Pole safety threshold

Vec3 T = ref_up.cross(N).normalized();
Vec3 B = N.cross(T);
Vec3 local_forward = B * std::cos(camYaw) + T * std::sin(camYaw);
Vec3 local_right = T * std::cos(camYaw) - B * std::sin(camYaw);

float moveSpeed = 15.0f * dt;
if (!orbit_mode) {
    // CRITICAL FIX 3: Use != GLFW_RELEASE to catch both initial press AND held key repeats
    if (glfwGetKey(window, GLFW_KEY_W) != GLFW_RELEASE) camPos = camPos + local_forward * moveSpeed;
    if (glfwGetKey(window, GLFW_KEY_S) != GLFW_RELEASE) camPos = camPos - local_forward * moveSpeed;
    if (glfwGetKey(window, GLFW_KEY_A) != GLFW_RELEASE) camPos = camPos - local_right * moveSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) != GLFW_RELEASE) camPos = camPos + local_right * moveSpeed;

    // Snap to terrain
    N = camPos.normalized();
    float noise_val = get_noise(N);
    float terrain_radius = planet_radius + noise_val * (planet_radius * 0.05f);
    camPos = N * (terrain_radius + 1.5f);
} else {
    if (glfwGetKey(window, GLFW_KEY_W) != GLFW_RELEASE) camPitch += 1.5f * dt;
    if (glfwGetKey(window, GLFW_KEY_S) != GLFW_RELEASE) camPitch -= 1.5f * dt;
    if (glfwGetKey(window, GLFW_KEY_A) != GLFW_RELEASE) camYaw -= 1.5f * dt;
    if (glfwGetKey(window, GLFW_KEY_D) != GLFW_RELEASE) camYaw += 1.5f * dt;
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

// CRITICAL FIX 4: Robust, Pole-Safe Chunk Streaming
N = camPos.normalized();
bool is_at_pole = (std::abs(N.y) > 0.98f);

if (!is_at_pole) {
    float current_theta = std::atan2(camPos.z, camPos.x);
    if (current_theta < 0) current_theta += 2.0f * M_PI;
    
    if (was_at_pole) {
        // Just left the pole: snap unwrapped_theta to current longitude 
        // to prevent massive delta_theta jumps from the singularity
        float two_pi = 2.0f * M_PI;
        float cycles = std::round(unwrapped_theta / two_pi);
        unwrapped_theta = current_theta + cycles * two_pi;
    } else {
        // Normal continuous tracking
        float delta_theta = current_theta - last_theta;
        if (delta_theta > M_PI) delta_theta -= 2.0f * M_PI;
        if (delta_theta < -M_PI) delta_theta += 2.0f * M_PI;
        unwrapped_theta += delta_theta;
    }
    last_theta = current_theta;
}
was_at_pole = is_at_pole; // Remember state for next frame

float current_phi = std::acos(std::max(-1.0f, std::min(1.0f, N.y)));

float chunk_x_float = (unwrapped_theta / (2.0f * M_PI)) * 16.0f;
float chunk_y_float = (current_phi / M_PI) * 8.0f;

int current_cx = std::floor(chunk_x_float);
int current_cy = std::floor(chunk_y_float);

int desired_origin_x = current_cx - holodeck.width / 2;
int desired_origin_y = current_cy - holodeck.height / 2;

int min_origin_y = 0;
int max_origin_y = 8 - holodeck.height;
desired_origin_y = std::max(min_origin_y, std::min(max_origin_y, desired_origin_y));

// CRITICAL FIX 5: SAFETY LIMIT on chunk shifting per frame
// Prevents the "death spiral" if desired_origin jumps for any reason
int max_shifts = 4; 
int shifts_x = 0, shifts_y = 0;

while (holodeck.origin_x < desired_origin_x && shifts_x < max_shifts) { holodeck.shift_x(1); shifts_x++; }
while (holodeck.origin_x > desired_origin_x && shifts_x < max_shifts) { holodeck.shift_x(-1); shifts_x++; }
while (holodeck.origin_y < desired_origin_y && shifts_y < max_shifts) { holodeck.shift_y(1); shifts_y++; }
while (holodeck.origin_y > desired_origin_y && shifts_y < max_shifts) { holodeck.shift_y(-1); shifts_y++; }
	
	

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
        
        float n = orbit_mode ? 1.0f : 0.1f;
        float f = orbit_mode ? 5000.0f : 250.0f; 
        float proj[16] = {
            fov/aspect, 0, 0, 0,
            0, fov, 0, 0,
            0, 0, -(f+n)/(f-n), -1.0f,
            0, 0, -(2.0f*f*n)/(f-n), 0
        };
        glLoadMatrixf(proj);

        glMatrixMode(GL_MODELVIEW); glLoadIdentity();

        if (orbit_mode) {
            Vec3 eye = camPos;
            Vec3 center(0, 0, 0);
            Vec3 up(0, 1, 0);
            Vec3 f_vec = (center - eye).normalized();
            Vec3 s = f_vec.cross(up).normalized();
            Vec3 u = s.cross(f_vec);
            float m[16] = { s.x, u.x, -f_vec.x, 0.0f, s.y, u.y, -f_vec.y, 0.0f, s.z, u.z, -f_vec.z, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
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
            Vec3 f_vec = (center - eye).normalized();
            Vec3 s_vec = f_vec.cross(view_up).normalized();
            Vec3 u_vec = s_vec.cross(f_vec);
            float m[16] = { s_vec.x, u_vec.x, -f_vec.x, 0.0f, s_vec.y, u_vec.y, -f_vec.y, 0.0f, s_vec.z, u_vec.z, -f_vec.z, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
            glMultMatrixf(m);
            glTranslatef(-eye.x, -eye.y, -eye.z);
        }

        for (int i = 0; i < holodeck.height; ++i) {
            for (int j = 0; j < holodeck.width; ++j) {
                HolodeckChunk* chunk = holodeck.grid[i][j];
                glCallList(chunk->dl);

                if (chunk->has_tardis) {
                    for (const auto& e : chunk->entities) {
                        glPushMatrix();
                        glTranslatef(e.local_pos.x, e.local_pos.y, e.local_pos.z);
                        
                        Vec3 up = e.local_pos.normalized();
                        Vec3 r_up(0, 1, 0);
                        if (std::abs(up.y) > 0.99f) r_up = Vec3(0, 0, 1);
                        
                        // FIX 1: Cross Product Order for Right-Handed Rule
                        Vec3 right = up.cross(r_up).normalized();
                        Vec3 forward = right.cross(up).normalized();

                        // FIX 2: Strict OpenGL Column-Major Memory Layout
                        float rotMat[16] = {
                            right.x, right.y, right.z, 0.0f,
                            up.x, up.y, up.z, 0.0f,
                            forward.x, forward.y, forward.z, 0.0f,
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