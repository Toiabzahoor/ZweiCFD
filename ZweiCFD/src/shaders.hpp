#pragma once

inline const char* lbm_compute_shader_src = R"(#version 430
layout(local_size_x = 8, local_size_y = 8, local_size_z = 8) in;

layout(std430, binding = 0) buffer F { float f[]; };
layout(std430, binding = 1) buffer FNew { float f_new[]; };
layout(std430, binding = 2) buffer IsSolid { int is_solid[]; };
layout(std430, binding = 3) buffer Rho { float rho[]; };
layout(std430, binding = 4) buffer U { vec4 u[]; };

uniform int NX;
uniform int NY;
uniform int NZ;
uniform float tau;
uniform float vx_inf;
uniform float vy_inf;
uniform float vz_inf;

const int cx[19] = int[](0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0, 0);
const int cy[19] = int[](0, 0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 1, -1, 1, -1);
const int cz[19] = int[](0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, -1, 1, 1, -1, -1, 1);
const float w[19] = float[](
    1.0/3.0,  1.0/18.0, 1.0/18.0, 1.0/18.0, 1.0/18.0,
    1.0/18.0, 1.0/18.0, 1.0/36.0, 1.0/36.0, 1.0/36.0,
    1.0/36.0, 1.0/36.0, 1.0/36.0, 1.0/36.0, 1.0/36.0,
    1.0/36.0, 1.0/36.0, 1.0/36.0, 1.0/36.0
);
const int opposite[19] = int[](
    0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15, 18, 17
);

int getIndex(int x, int y, int z, int q) {
    return (z * NY * NX + y * NX + x) * 19 + q;
}

int getScalarIndex(int x, int y, int z) {
    return z * NY * NX + y * NX + x;
}

void main() {
    ivec3 pos = ivec3(gl_GlobalInvocationID.xyz);
    if (pos.x >= NX || pos.y >= NY || pos.z >= NZ) return;
    
    int x = pos.x;
    int y = pos.y;
    int z = pos.z;
    int s_idx = getScalarIndex(x, y, z);
    
    bool is_boundary = (x == 0 || x == NX-1 || y == 0 || y == NY-1 || z == 0 || z == NZ-1);
    
    if (is_boundary) {
        rho[s_idx] = 1.0;
        u[s_idx] = vec4(vx_inf, vy_inf, vz_inf, 0.0);
        float usq = vx_inf*vx_inf + vy_inf*vy_inf + vz_inf*vz_inf;
        for (int q = 0; q < 19; ++q) {
            float cu = cx[q]*vx_inf + cy[q]*vy_inf + cz[q]*vz_inf;
            float feq = w[q] * (1.0 + 3.0*cu + 4.5*cu*cu - 1.5*usq);
            
            int nx = x + cx[q];
            int ny = y + cy[q];
            int nz = z + cz[q];
            if (nx >= 0 && nx < NX && ny >= 0 && ny < NY && nz >= 0 && nz < NZ) {
                if (is_solid[getScalarIndex(nx, ny, nz)] == 0) {
                    f_new[getIndex(nx, ny, nz, q)] = feq;
                }
            }
        }
    } else if (is_solid[s_idx] == 0) {
        // 1. Compute macro from f
        float density = 0.0;
        vec3 vel = vec3(0.0);
        for (int q = 0; q < 19; ++q) {
            float val = f[getIndex(x, y, z, q)];
            density += val;
            vel.x += val * cx[q];
            vel.y += val * cy[q];
            vel.z += val * cz[q];
        }
        
        rho[s_idx] = density;
        vec4 u_val = vec4(0.0);
        if (density > 1e-9) {
            u_val = vec4(vel / density, 0.0);
        }
        u[s_idx] = u_val;
        
        // 2. Collide
        float usq = u_val.x*u_val.x + u_val.y*u_val.y + u_val.z*u_val.z;
        float omega = 1.0 / tau;
        
        for (int q = 0; q < 19; ++q) {
            float cu = cx[q]*u_val.x + cy[q]*u_val.y + cz[q]*u_val.z;
            float feq = w[q] * density * (1.0 + 3.0*cu + 4.5*cu*cu - 1.5*usq);
            
            float f_post = f[getIndex(x, y, z, q)] - omega * (f[getIndex(x, y, z, q)] - feq);
            
            // 3. Stream
            int nx = x + cx[q];
            int ny = y + cy[q];
            int nz = z + cz[q];
            
            if (nx >= 0 && nx < NX && ny >= 0 && ny < NY && nz >= 0 && nz < NZ) {
                if (is_solid[getScalarIndex(nx, ny, nz)] == 0) {
                    f_new[getIndex(nx, ny, nz, q)] = f_post;
                } else {
                    f_new[getIndex(x, y, z, opposite[q])] = f_post;
                }
            } else {
                f_new[getIndex(x, y, z, opposite[q])] = f_post;
            }
        }
    } else {
        // Solid nodes update macro to 0 for rendering
        rho[s_idx] = 1.0;
        u[s_idx] = vec4(0.0);
    }
}
)";

inline const char* particle_update_comp = R"(#version 430
layout(local_size_x = 256) in;

struct Particle {
    vec2 pos;
    vec2 prevPos;
    float baseSize;
    float alpha;
    float speedJitter;
    float velocityMag;
    int isActive;
    float age;
};

layout(std430, binding = 5) buffer Particles {
    Particle p[];
};

layout(std430, binding = 4) buffer U { vec4 u[]; };
layout(std430, binding = 2) buffer IsSolid { int is_solid[]; };
layout(std430, binding = 6) buffer LVPMGrid { vec2 lvpm_u[]; };

uniform int maxParticles;
uniform float dt;
uniform float renderScale;
uniform int current_sim;
uniform float v_inf;
uniform float alpha_angle;
uniform int windDirection;
uniform int gridNX;
uniform int gridNY;
uniform float screenWidth;
uniform float screenHeight;

uniform vec2 spawnTL;
uniform vec2 spawnBR;
uniform vec2 killTL;
uniform vec2 killBR;

float rand(vec2 co){
    return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);
}

void main() {
    uint idx = gl_GlobalInvocationID.x;
    if (idx >= maxParticles) return;
    
    if (p[idx].isActive == 0) return;
    
    p[idx].prevPos = p[idx].pos;
    p[idx].age += dt;
    
    vec2 physPos = vec2(p[idx].pos.x / renderScale, -p[idx].pos.y / renderScale);
    vec2 vel = vec2(0.0);
    bool inside = false;
    
    if (current_sim == 1) { // LBM
        float x_exact = physPos.x * 20.0 + 32.0;
        float y_exact = -physPos.y * 20.0 + 16.0;
        int z = 16;
        
        int i0 = int(floor(x_exact));
        int j0 = int(floor(y_exact));
        
        int cx = int(round(x_exact));
        int cy = int(round(y_exact));
        
        if (cx >= 0 && cx < gridNX && cy >= 0 && cy < gridNY) {
            // Do not kill particles here! Let them slow down in the solid voxels (vel=0)
            // and get masked by the geometric polygon drawn over the airfoil.
        }
        
        if (i0 >= 0 && i0 < gridNX - 1 && j0 >= 0 && j0 < gridNY - 1) {
            int idx00 = z * gridNY * gridNX + j0 * gridNX + i0;
            int idx10 = z * gridNY * gridNX + j0 * gridNX + (i0 + 1);
            int idx01 = z * gridNY * gridNX + (j0 + 1) * gridNX + i0;
            int idx11 = z * gridNY * gridNX + (j0 + 1) * gridNX + (i0 + 1);
            
            float tx = x_exact - float(i0);
            float ty = y_exact - float(j0);
            
            vec2 v00 = vec2(u[idx00].x, u[idx00].y);
            vec2 v10 = vec2(u[idx10].x, u[idx10].y);
            vec2 v01 = vec2(u[idx01].x, u[idx01].y);
            vec2 v11 = vec2(u[idx11].x, u[idx11].y);
            
            float lbm_scale = v_inf / max(0.0001, 0.05); // using base reference
            vec2 interp_vel = (1.0 - tx) * (1.0 - ty) * v00 + 
                              tx * (1.0 - ty) * v10 + 
                              (1.0 - tx) * ty * v01 + 
                              tx * ty * v11;
                              
            vel = interp_vel * lbm_scale;
        } else {
            float vx = v_inf;
            float vy = 0.0;
            if (windDirection == 1) { vx = -vx; vy = -vy; }
            else if (windDirection == 2) { float t = vx; vx = vy; vy = -t; }
            else if (windDirection == 3) { float t = vx; vx = -vy; vy = t; }
            vel = vec2(vx, vy);
        }
    } else { // Panel method
        // map physPos to LVPM grid range (-3.0 to 4.0 in X, -3.0 to 3.0 in Y, 200x200)
        float minX = -3.0, maxX = 4.0;
        float minY = -3.0, maxY = 3.0;
        int lvpmNX = 200, lvpmNY = 200;
        
        if (physPos.x >= minX && physPos.x <= maxX && physPos.y >= minY && physPos.y <= maxY) {
            float x_idx = (physPos.x - minX) / ((maxX - minX) / (lvpmNX - 1.0));
            float y_idx = (physPos.y - minY) / ((maxY - minY) / (lvpmNY - 1.0));
            int i0 = int(floor(x_idx));
            int j0 = int(floor(y_idx));
            i0 = clamp(i0, 0, lvpmNX - 2);
            j0 = clamp(j0, 0, lvpmNY - 2);
            
            int i1 = i0 + 1;
            int j1 = j0 + 1;
            float tx = x_idx - float(i0);
            float ty = y_idx - float(j0);
            
            vec2 v00 = lvpm_u[j0 * lvpmNX + i0];
            vec2 v10 = lvpm_u[j0 * lvpmNX + i1];
            vec2 v01 = lvpm_u[j1 * lvpmNX + i0];
            vec2 v11 = lvpm_u[j1 * lvpmNX + i1];
            
            vel = (1.0 - tx) * (1.0 - ty) * v00 + 
                  tx * (1.0 - ty) * v10 + 
                  (1.0 - tx) * ty * v01 + 
                  tx * ty * v11;
        } else {
            float vx = v_inf * cos(alpha_angle * 3.14159 / 180.0);
            float vy = v_inf * sin(alpha_angle * 3.14159 / 180.0);
            if (windDirection == 1) { vx = -vx; vy = -vy; }
            else if (windDirection == 2) { float t = vx; vx = vy; vy = -t; }
            else if (windDirection == 3) { float t = vx; vx = -vy; vy = t; }
            vel = vec2(vx, vy);
        }
    }
    
    if (inside) {
        p[idx].isActive = 0;
        return;
    }
    
    p[idx].velocityMag = length(vel);
    
    float baseSpeed = 300.0;
    float stepScale = baseSpeed * p[idx].speedJitter * dt / max(0.1, v_inf);
    p[idx].pos.x += vel.x * stepScale;
    p[idx].pos.y -= vel.y * stepScale;
    
    if (p[idx].pos.x < killTL.x || p[idx].pos.x > killBR.x ||
        p[idx].pos.y < killTL.y || p[idx].pos.y > killBR.y) {
        p[idx].isActive = 0;
    }
}
)";

inline const char* particle_render_vs = R"(#version 430
in vec3 vertexPosition;
in vec2 vertexTexCoord;

struct Particle {
    vec2 pos;
    vec2 prevPos;
    float baseSize;
    float alpha;
    float speedJitter;
    float velocityMag;
    int isActive;
    float age;
};

uniform mat4 mvp; // raylib custom
uniform vec2 killTL;
uniform vec2 killBR;
uniform float v_inf;

layout(std430, binding = 5) buffer Particles {
    Particle p[];
};

out vec4 fragColor;
out vec2 fragTexCoord;

void main() {
    Particle pt = p[gl_InstanceID];
    
    vec2 trailDir = pt.pos - pt.prevPos;
    float len = length(trailDir);
    vec2 dir = vec2(1.0, 0.0);
    if (len > 0.0001) {
        dir = trailDir / len;
    }
    vec2 normal = vec2(-dir.y, dir.x);
    
    // trailEnd is the tail (behind the head)
    vec2 trailEnd = pt.pos - dir * max(len * 4.0, 2.0); 
    
    // Map Left vertices (x < 0.5) to the Tail, Right vertices (x > 0.5) to the Head
    // This preserves the quad's winding order so backface culling doesn't discard it!
    vec2 finalPos = trailEnd;
    if (vertexTexCoord.x > 0.5) finalPos = pt.pos;
    
    float thickness = max(0.5, pt.baseSize * 1.5);
    if (vertexTexCoord.y > 0.5) finalPos += normal * thickness;
    else finalPos -= normal * thickness;
    
    gl_Position = mvp * vec4(finalPos, 0.0, 1.0);
      // Blue -> Green -> Maroon -> Red
      float normV = clamp(pt.velocityMag / v_inf, 0.0, 1.5) / 1.5;
      vec3 colBlue = vec3(0.0, 0.0, 1.0);
      vec3 colGreen = vec3(0.0, 1.0, 0.0);
      vec3 colMaroon = vec3(0.5, 0.0, 0.0);
      vec3 colRed = vec3(1.0, 0.0, 0.0);
      
      vec3 baseColor;
      if (normV < 0.33) {
          baseColor = mix(colBlue, colGreen, normV / 0.33);
      } else if (normV < 0.66) {
          baseColor = mix(colGreen, colMaroon, (normV - 0.33) / 0.33);
      } else {
          baseColor = mix(colMaroon, colRed, (normV - 0.66) / 0.34);
      }
      
      float fadeIn = clamp(pt.age * 2.0, 0.0, 1.0);
      float finalAlpha = pt.alpha * fadeIn;
      
      fragColor = vec4(baseColor, finalAlpha);
    fragTexCoord = vertexTexCoord;
}
)";

inline const char* particle_render_fs = R"(#version 430
in vec4 fragColor;
in vec2 fragTexCoord;
out vec4 finalColor;

void main() {
    finalColor = fragColor;
}
)";
