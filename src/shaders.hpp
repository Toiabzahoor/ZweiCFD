#pragma once

inline const char *lbm_compute_shader_src = R"(#version 430
layout(local_size_x = 8, local_size_y = 8, local_size_z = 8) in;

layout(std430, binding = 0) buffer F { float f[]; };
layout(std430, binding = 1) buffer FNew { float f_new[]; };
layout(std430, binding = 2) buffer SDF { float sdf[]; };
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
                if (sdf[getScalarIndex(nx, ny, nz)] > 0.0) {
                    f_new[getIndex(nx, ny, nz, q)] = feq;
                }
            }
        }
    } else if (sdf[s_idx] > 0.0) {
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
        
        float usq = u_val.x*u_val.x + u_val.y*u_val.y + u_val.z*u_val.z;
        
        float Pi_xx = 0.0, Pi_yy = 0.0, Pi_zz = 0.0;
        float Pi_xy = 0.0, Pi_xz = 0.0, Pi_yz = 0.0;
        for (int q = 0; q < 19; ++q) {
            float f_q = f[getIndex(x, y, z, q)];
            float cu = cx[q]*u_val.x + cy[q]*u_val.y + cz[q]*u_val.z;
            float feq = w[q] * density * (1.0 + 3.0*cu + 4.5*cu*cu - 1.5*usq);
            float f_neq = f_q - feq;
            Pi_xx += f_neq * cx[q] * cx[q];
            Pi_yy += f_neq * cy[q] * cy[q];
            Pi_zz += f_neq * cz[q] * cz[q];
            Pi_xy += f_neq * cx[q] * cy[q];
            Pi_xz += f_neq * cx[q] * cz[q];
            Pi_yz += f_neq * cy[q] * cz[q];
        }
        float Pi_mag = sqrt(Pi_xx*Pi_xx + Pi_yy*Pi_yy + Pi_zz*Pi_zz + 2.0*(Pi_xy*Pi_xy + Pi_xz*Pi_xz + Pi_yz*Pi_yz));
        float C_smag = 0.16;
        float tau_eff = 0.5 * (tau + sqrt(tau*tau + 18.0 * C_smag*C_smag * Pi_mag / max(density, 1e-9)));
        float omega = 1.0 / tau_eff;
        
        for (int q = 0; q < 19; ++q) {
            float cu = cx[q]*u_val.x + cy[q]*u_val.y + cz[q]*u_val.z;
            float feq = w[q] * density * (1.0 + 3.0*cu + 4.5*cu*cu - 1.5*usq);
            
            float f_post = f[getIndex(x, y, z, q)] - omega * (f[getIndex(x, y, z, q)] - feq);
            
            int nx = x + cx[q];
            int ny = y + cy[q];
            int nz = z + cz[q];
            
            if (nx >= 0 && nx < NX && ny >= 0 && ny < NY && nz >= 0 && nz < NZ) {
                if (sdf[getScalarIndex(nx, ny, nz)] > 0.0) {
                    f_new[getIndex(nx, ny, nz, q)] = f_post;
                } else {
                    float sdf_x = sdf[s_idx];
                    float sdf_nx = sdf[getScalarIndex(nx, ny, nz)];
                    float q_dist = sdf_x / max(1e-6, sdf_x - sdf_nx);
                    
                    int prev_x = x - cx[q];
                    int prev_y = y - cy[q];
                    int prev_z = z - cz[q];
                    
                    float f_post_prev = f_post; // Fallback
                    if (prev_x >= 0 && prev_x < NX && prev_y >= 0 && prev_y < NY && prev_z >= 0 && prev_z < NZ) {
                        if (sdf[getScalarIndex(prev_x, prev_y, prev_z)] > 0.0) {
                            f_post_prev = f[getIndex(prev_x, prev_y, prev_z, q)]; // Use pre-collision f as approximation for stability
                        }
                    }
                    
                    float bounced = 0.0;
                    if (q_dist >= 0.5) {
                        bounced = ((2.0 * q_dist - 1.0) / (2.0 * q_dist)) * f_post + (1.0 / (2.0 * q_dist)) * f_post_prev;
                    } else {
                        bounced = (2.0 * q_dist) * f_post + (1.0 - 2.0 * q_dist) * f_post_prev;
                    }
                    f_new[getIndex(x, y, z, opposite[q])] = bounced;
                }
            } else {
                f_new[getIndex(x, y, z, opposite[q])] = f_post;
            }
        }
    } else {
        rho[s_idx] = 1.0;
        u[s_idx] = vec4(0.0);
    }
}
)";

inline const char *particle_update_comp = R"(#version 430
layout(local_size_x = 256) in;

struct Particle {
    vec3 pos;
    float pad1;
    vec3 prevPos;
    float pad2;
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
layout(std430, binding = 2) buffer SDF { float sdf[]; };
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
uniform int gridNZ;
uniform float gridScale;
uniform float screenWidth;
uniform float screenHeight;

uniform vec3 spawnTL;
uniform vec3 spawnBR;
uniform vec3 killTL;
uniform vec3 killBR;
uniform float baseSpeed;
uniform int has_lvpm;

float rand(vec2 co){
    return fract(sin(dot(co.xy ,vec2(12.9898,78.233))) * 43758.5453);
}

vec3 getVelocity(vec3 physPos, out bool inside) {
    vec3 vel = vec3(0.0);
    inside = false;
    
    if (current_sim == 1) {
        float x_exact = physPos.x * gridScale + float(gridNX) / 2.0;
        float y_exact = physPos.y * gridScale + float(gridNY) / 2.0;
        float z_exact = physPos.z * gridScale + float(gridNZ) / 2.0;
        
        int i0 = int(floor(x_exact));
        int j0 = int(floor(y_exact));
        int k0 = int(floor(z_exact));
        
        if (i0 >= 0 && i0 < gridNX - 1 && j0 >= 0 && j0 < gridNY - 1 && k0 >= 0 && k0 < gridNZ - 1) {
            float tx = x_exact - float(i0);
            float ty = y_exact - float(j0);
            float tz = z_exact - float(k0);
            
            int i1 = i0 + 1;
            int j1 = j0 + 1;
            int k1 = k0 + 1;
            
            int i000 = k0 * gridNY * gridNX + j0 * gridNX + i0;
            int i100 = k0 * gridNY * gridNX + j0 * gridNX + i1;
            int i010 = k0 * gridNY * gridNX + j1 * gridNX + i0;
            int i110 = k0 * gridNY * gridNX + j1 * gridNX + i1;
            int i001 = k1 * gridNY * gridNX + j0 * gridNX + i0;
            int i101 = k1 * gridNY * gridNX + j0 * gridNX + i1;
            int i011 = k1 * gridNY * gridNX + j1 * gridNX + i0;
            int i111 = k1 * gridNY * gridNX + j1 * gridNX + i1;
            
            float interp_sdf = 
                (1.0-tx)*(1.0-ty)*(1.0-tz)*sdf[i000] + tx*(1.0-ty)*(1.0-tz)*sdf[i100] +
                (1.0-tx)*ty*(1.0-tz)*sdf[i010] + tx*ty*(1.0-tz)*sdf[i110] +
                (1.0-tx)*(1.0-ty)*tz*sdf[i001] + tx*(1.0-ty)*tz*sdf[i101] +
                (1.0-tx)*ty*tz*sdf[i011] + tx*ty*tz*sdf[i111];
            
            if (interp_sdf <= 0.0) {
                inside = true;
            }
            
            vec3 v000 = u[i000].xyz; vec3 v100 = u[i100].xyz;
            vec3 v010 = u[i010].xyz; vec3 v110 = u[i110].xyz;
            vec3 v001 = u[i001].xyz; vec3 v101 = u[i101].xyz;
            vec3 v011 = u[i011].xyz; vec3 v111 = u[i111].xyz;
            
            vec3 interp_vel = 
                (1.0-tx)*(1.0-ty)*(1.0-tz)*v000 + tx*(1.0-ty)*(1.0-tz)*v100 +
                (1.0-tx)*ty*(1.0-tz)*v010 + tx*ty*(1.0-tz)*v110 +
                (1.0-tx)*(1.0-ty)*tz*v001 + tx*(1.0-ty)*tz*v101 +
                (1.0-tx)*ty*tz*v011 + tx*ty*tz*v111;
                
            float lbm_scale = v_inf / max(0.0001, 0.05); 
            vel = interp_vel * lbm_scale;
            
            if (isnan(vel.x) || isnan(vel.y) || isnan(vel.z) || isinf(vel.x) || isinf(vel.y) || isinf(vel.z)) {
                vel = vec3(0.0);
                inside = true;
            } else if (length(vel) > v_inf * 5.0) {
                vel = normalize(vel) * v_inf * 5.0;
            }
        } else {
            float vx = v_inf;
            float vy = 0.0;
            float vz = 0.0;
            if (windDirection == 1) { vx = -vx; vy = -vy; }
            else if (windDirection == 2) { float t = vx; vx = vy; vy = -t; }
            else if (windDirection == 3) { float t = vx; vx = -vy; vy = t; }
            vel = vec3(vx, vy, vz);
        }
    } else {
        float minX = -3.0;
        float maxX = 4.0;
        float minY = -3.0;
        float maxY = 3.0;
        int nx = 200;
        int ny = 200;
        
        float px = physPos.x;
        float py = physPos.y;
        
        if (has_lvpm == 1 && px >= minX && px <= maxX && py >= minY && py <= maxY) {
            float x_idx = (px - minX) / ((maxX - minX) / float(nx - 1));
            float y_idx = (py - minY) / ((maxY - minY) / float(ny - 1));
            
            int i0 = int(floor(x_idx));
            int j0 = int(floor(y_idx));
            
            if (i0 < 0) i0 = 0; if (i0 >= nx - 1) i0 = nx - 2;
            if (j0 < 0) j0 = 0; if (j0 >= ny - 1) j0 = ny - 2;
            
            float tx = x_idx - float(i0);
            float ty = y_idx - float(j0);
            
            vec2 v00 = lvpm_u[j0 * nx + i0];
            vec2 v10 = lvpm_u[j0 * nx + i0 + 1];
            vec2 v01 = lvpm_u[(j0 + 1) * nx + i0];
            vec2 v11 = lvpm_u[(j0 + 1) * nx + i0 + 1];
            
            vec2 interp_vel = 
                (1.0 - tx) * (1.0 - ty) * v00 + 
                tx * (1.0 - ty) * v10 + 
                (1.0 - tx) * ty * v01 + 
                tx * ty * v11;
                
            vel = vec3(interp_vel.x, interp_vel.y, 0.0);
        } else {
            float vx = v_inf * cos(alpha_angle * 3.14159 / 180.0);
            float vy = v_inf * sin(alpha_angle * 3.14159 / 180.0);
            
            if (windDirection == 1) { vx = -vx; vy = -vy; }
            else if (windDirection == 2) { float t = vx; vx = vy; vy = -t; }
            else if (windDirection == 3) { float t = vx; vx = -vy; vy = t; }
            
            vel = vec3(vx, vy, 0.0);
        }
    }
    return vel;
}

uint hash(uint x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

float randomFloat(uint seed) {
    return float(hash(seed)) / 4294967295.0;
}

void main() {
    uint idx = gl_GlobalInvocationID.x;
    if (idx >= maxParticles) return;
    
    if (p[idx].isActive == 0) {
        p[idx].isActive = 1;
        p[idx].age = 0.0;
        
        float rx = randomFloat(idx * 3u);
        float ry = randomFloat(idx * 3u + 1u);
        float rz = randomFloat(idx * 3u + 2u);
        
        p[idx].baseSize = 0.02 + randomFloat(idx * 4u) * 0.05;
        p[idx].alpha = 0.05 + randomFloat(idx * 4u + 1u) * 0.15;
        p[idx].speedJitter = 0.5 + randomFloat(idx * 4u + 2u) * 1.0;
        p[idx].pos.x = spawnTL.x + rx * (spawnBR.x - spawnTL.x);
        p[idx].pos.y = spawnTL.y + ry * (spawnBR.y - spawnTL.y);
        p[idx].pos.z = spawnTL.z + rz * (spawnBR.z - spawnTL.z);
        
        p[idx].prevPos = p[idx].pos;
        return;
    }
    
    p[idx].prevPos = p[idx].pos;
    p[idx].age += dt;
    
    int numSubSteps = 5;
    float subDt = dt / float(numSubSteps);
    
    for (int step = 0; step < numSubSteps; ++step) {
        vec3 physPos = vec3(p[idx].pos.x / renderScale, -p[idx].pos.y / renderScale, p[idx].pos.z / renderScale);
        bool inside = false;
        vec3 vel = getVelocity(physPos, inside);
        
        if (inside) {
            p[idx].isActive = 0;
            return;
        }
        
        if (step == 0) {
            p[idx].velocityMag = length(vel);
        }
        
        float stepScale = baseSpeed * p[idx].speedJitter * subDt;
        p[idx].pos.x += vel.x * stepScale;
        p[idx].pos.y -= vel.y * stepScale;
        
        float zTurbulence = (randomFloat(idx * 7u + uint(p[idx].age * 100.0)) - 0.5) * 20.0;
        p[idx].pos.z += zTurbulence * subDt;
        p[idx].pos.z += vel.z * stepScale;
        
        if (p[idx].pos.x < killTL.x || p[idx].pos.x > killBR.x ||
            p[idx].pos.y < killTL.y || p[idx].pos.y > killBR.y ||
            p[idx].pos.z < killTL.z || p[idx].pos.z > killBR.z) {
            
            float ry = randomFloat(idx * 6u + uint(p[idx].age * 1000.0));
            float rz = randomFloat(idx * 6u + 1u + uint(p[idx].age * 1000.0));
            
            if (windDirection == 0 || windDirection == 1) {
                if (p[idx].pos.y < killTL.y || p[idx].pos.y > killBR.y || p[idx].pos.z < killTL.z || p[idx].pos.z > killBR.z) {
                    p[idx].isActive = 0;
                    return;
                }
                
                p[idx].pos.y = spawnTL.y + ry * (spawnBR.y - spawnTL.y);
                p[idx].pos.z = spawnTL.z + rz * (spawnBR.z - spawnTL.z);
                
                if (windDirection == 0) p[idx].pos.x = spawnTL.x;
                else p[idx].pos.x = spawnBR.x;
            } else {
                if (p[idx].pos.x < killTL.x || p[idx].pos.x > killBR.x || p[idx].pos.z < killTL.z || p[idx].pos.z > killBR.z) {
                    p[idx].isActive = 0;
                    return;
                }
                
                p[idx].pos.x = spawnTL.x + ry * (spawnBR.x - spawnTL.x);
                p[idx].pos.z = spawnTL.z + rz * (spawnBR.z - spawnTL.z);
                
                if (windDirection == 2) p[idx].pos.y = spawnBR.y;
                else p[idx].pos.y = spawnTL.y;
            }
            
            p[idx].prevPos = p[idx].pos;
            p[idx].age = 0.0;
            return;
        }
    }
}
)";

inline const char *particle_render_vs = R"(#version 430
in vec3 vertexPosition;
in vec2 vertexTexCoord;

struct Particle {
    vec3 pos;
    float pad1;
    vec3 prevPos;
    float pad2;
    float baseSize;
    float alpha;
    float speedJitter;
    float velocityMag;
    int isActive;
    float age;
};

uniform mat4 mvp;  
uniform vec3 killTL;
uniform vec3 killBR;
uniform float v_inf;

layout(std430, binding = 5) buffer Particles {
    Particle p[];
};

out vec4 fragColor;
out vec2 fragTexCoord;

void main() {
    Particle pt = p[gl_InstanceID];
    
    if (pt.isActive == 0) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0); // Cull outside NDC
        return;
    }
    
    vec3 trailDir = pt.pos - pt.prevPos;
    float len = length(trailDir);
    vec3 dir = vec3(1.0, 0.0, 0.0);
    if (len > 0.0001) {
        dir = trailDir / len;
    }
    
    float trailLength = min(len * 1.5, 20.0);
    vec3 trailEnd = pt.pos - dir * max(trailLength, 2.0); 
    
    vec3 finalPos3D = (vertexTexCoord.x > 0.5) ? pt.pos : trailEnd;
    vec4 clipPos = mvp * vec4(finalPos3D, 1.0);
    
    vec4 nextClipPos = mvp * vec4(finalPos3D + dir, 1.0);
    vec2 clipDir = normalize(nextClipPos.xy / nextClipPos.w - clipPos.xy / clipPos.w + vec2(0.0001)); // add tiny epsilon
    vec2 clipNormal = vec2(-clipDir.y, clipDir.x);
    
    float thickness = max(0.5, pt.baseSize * 1.5);
    
    if (vertexTexCoord.y > 0.5) clipPos.xy += clipNormal * thickness * 0.003 * clipPos.w;
    else clipPos.xy -= clipNormal * thickness * 0.003 * clipPos.w;
    
    gl_Position = clipPos;
    float normV = clamp(pt.velocityMag / max(0.001, v_inf), 0.0, 1.5) / 1.5;
    vec3 colBlue = vec3(0.0, 0.0, 1.0);
    vec3 colGreen = vec3(0.0, 1.0, 1.0);
    vec3 colMaroon = vec3(1.0, 0.0, 1.0);
    vec3 colRed = vec3(1.0, 0.8, 0.9);
    
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

inline const char *particle_render_fs = R"(#version 430
in vec2 fragTexCoord;
in vec4 fragColor;
out vec4 finalColor;

uniform sampler2D texture0;

void main() {
    vec4 texelColor = texture(texture0, fragTexCoord);
    float alphaMask = texelColor.r; 
    
    vec4 col = fragColor;
    col.a *= alphaMask;
    
    finalColor = col;
}
)";
