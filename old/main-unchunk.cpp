#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <chrono>
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
// 2. 3-Octave FBM Noise Generation
// ============================================================================
float base_noise_3d(const Vec3& p) {
    float n1 = std::sin(p.dot(Vec3(12.9898f, 78.233f, 45.164f))) * 43758.5453f;
    float n2 = std::sin(p.dot(Vec3(39.346f, 11.135f, 92.123f))) * 23421.631f;
    float n3 = std::sin(p.dot(Vec3(73.156f, 52.235f, 9.151f))) * 15782.153f;
    
    float val1 = n1 - std::floor(n1);
    float val2 = n2 - std::floor(n2);
    float val3 = n3 - std::floor(n3);
    
    return (val1 + val2 + val3) / 3.0f;
}

float get_noise(const Vec3& dir) {
    float value = 0.0f;
    float amplitude = 0.6f;
    float frequency = 3.0f; // Base frequency
    float max_amp = 0.0f;
    
    // 3 Octaves
    for (int i = 0; i < 3; ++i) {
        value += amplitude * base_noise_3d(dir * frequency);
        max_amp += amplitude;
        frequency *= 2.0f;
        amplitude *= 0.5f;
    }
    return value / max_amp; // Normalized strictly to 0..1
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
        glShadeModel(GL_FLAT); // Flat shading for stylized low-poly look
        
        glBegin(GL_TRIANGLES);
        for (const auto& tri : mesh) {
            glNormal3f(tri.normal.x, tri.normal.y, tri.normal.z);
            
            float h0 = std::max(0.0f, std::min(1.0f, (tri.v0.length() - planet_radius) / (planet_radius * 0.05f)));
            float h1 = std::max(0.0f, std::min(1.0f, (tri.v1.length() - planet_radius) / (planet_radius * 0.05f)));
            float h2 = std::max(0.0f, std::min(1.0f, (tri.v2.length() - planet_radius) / (planet_radius * 0.05f)));
            
            auto set_color = [&](float h) {
                if (h < 0.25f) glColor3f(0.1f, 0.3f, 0.7f); // Deep Water
                else if (h < 0.3f) glColor3f(0.2f, 0.4f, 0.8f); // Shallow Water
                else if (h < 0.4f) glColor3f(0.76f, 0.70f, 0.50f); // Sand
                else if (h < 0.75f) glColor3f(0.13f, 0.54f, 0.13f); // Grass
                else glColor3f(0.9f, 0.9f, 0.95f); // Snow
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
        
        float theta_base = (float(cx) / total_chunks_x) * 2.0f * M_PI; 
        float phi_base = (float(cy) / total_chunks_y) * M_PI;          

        std::vector<Vec3> verts;
        int res = 32; // Increased resolution to fix equator texture blurring
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
    }

    void check_for_tardis() {
        if (cx == 8 && cy == 4) {
            has_tardis = true;
            
            // Calculate exact mathematical center of this chunk
            float total_chunks_x = 16.0f;
            float total_chunks_y = 8.0f;
            float center_theta = (float(cx) + 0.5f) / total_chunks_x * 2.0f * M_PI;
            float center_phi = (float(cy) + 0.5f) / total_chunks_y * M_PI;
            
            Vec3 tardis_dir(std::sin(center_phi) * std::cos(center_theta), std::cos(center_phi), std::sin(center_phi) * std::sin(center_theta));
            
            // Snap precisely to the noise terrain surface
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
                    
                    Vec3 tardis_pos = entities[0].local_pos;
                    float dist_to_house = (e.local_pos - tardis_pos).length();
                    if (dist_to_house > 5.0f) {
                        e.velocity = (tardis_pos - e.local_pos).normalized() * 1.5f;
                    }
                }
            }
        }
    }
};

// ============================================================================
// 5. The Planetary Holodeck
// ============================================================================
class PlanetaryHolodeck {
public:
    int width = 16, height = 8;
    std::vector<std::vector<HolodeckChunk*>> grid;
    float planet_radius;

    // By loading the full 16x8 closed sphere, we completely eliminate edge streaming bugs
    PlanetaryHolodeck(float r) : planet_radius(r) {
        grid.resize(height);
        for (int i = 0; i < height; ++i) {
            grid[i].resize(width);
            for (int j = 0; j < width; ++j) {
                grid[i][j] = new HolodeckChunk(j, i, r);
            }
        }
    }

    ~PlanetaryHolodeck() {
        for (auto& row : grid) for (auto* chunk : row) delete chunk;
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
        if (e.type == LogicalEntity::HUMANOID) e.local_pos = entities[0].local_pos + Vec3(2.0f, 0, 0); 
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

    std::cout << "Initializing Closed Planetary Sphere...\n";
    float planet_radius = 50.0f;
    PlanetaryHolodeck holodeck(planet_radius); 

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glShadeModel(GL_FLAT);

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

    std::cout << "\n--- HOLODECK CONTROLS ---\n";
    std::cout << "Mouse: Look around (FPS)\n";
    std::cout << "W/A/S/D: Walk on Planet Surface\n";
    std::cout << "Z: Toggle Orbit Mode (Zoom out to see planet)\n";
    std::cout << "PageUp/PageDown: Zoom In/Out (Orbit Mode)\n";
    std::cout << "T: Advance Time | B: BACKUP | R: RESTORE\n";
    std::cout << "ESC: Quit\n\n";

    auto lastTime = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window)) {
        auto now = std::chrono::high_resolution_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;

        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) glfwSetWindowShouldClose(window, true);

        float moveSpeed = 15.0f * dt;
        if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
            orbit_mode = !orbit_mode;
            if (orbit_mode) glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            else glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            while(glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) glfwPollEvents();
        }
        
        if (glfwGetKey(window, GLFW_KEY_PAGE_UP) == GLFW_PRESS) orbit_dist -= 150.0f * dt;
        if (glfwGetKey(window, GLFW_KEY_PAGE_DOWN) == GLFW_PRESS) orbit_dist += 150.0f * dt;
        if (orbit_dist < planet_radius * 1.2f) orbit_dist = planet_radius * 1.2f;
        if (orbit_dist > 3000.0f) orbit_dist = 3000.0f; // Massive zoom out distance

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
        
        // Correct projection matrix with massive far plane (f=5000) for zooming out
        float n = 0.1f;
        float f = 5000.0f; 
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
            s = f_vec.cross(view_up).normalized();
            u = s.cross(f_vec);
            float m[16] = { s.x, u.x, -f_vec.x, 0.0f, s.y, u.y, -f_vec.y, 0.0f, s.z, u.z, -f_vec.z, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
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