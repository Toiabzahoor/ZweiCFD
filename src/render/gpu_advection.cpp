#include "ZweiCFD/render/gpu_advection.hpp"
#include <iostream>
#include <random>
#include <cmath>

namespace zweicfd {

static const char* computeShaderSource = R"(#version 430 core
layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

struct Particle {
    vec4 pos_age;
    vec4 vel_life;
};

layout(std430, binding = 0) buffer ParticleBuffer {
    Particle particles[];
};

uniform sampler3D uVelocityField;
uniform vec3 uDomainMin;
uniform vec3 uDomainMax;
uniform float uDeltaTime;
uniform vec3 uInletOrigin;
uniform vec2 uInletSize;
uniform uint uSeed;

float hash(uint n) {
    n = (n << 13U) ^ n;
    n = n * (n * n * 15731U + 789221U) + 1376312589U;
    return float(n & 0x7fffffffU) / float(0x7fffffff);
}

vec3 sampleVelocity(vec3 pos) {
    vec3 uvw = (pos - uDomainMin) / (uDomainMax - uDomainMin);
    if (any(lessThan(uvw, vec3(0.0))) || any(greaterThan(uvw, vec3(1.0)))) {
        return vec3(0.0);
    }
    return texture(uVelocityField, uvw).xyz;
}

vec3 rk4Step(vec3 p, float dt) {
    vec3 k1 = sampleVelocity(p);
    vec3 k2 = sampleVelocity(p + 0.5 * dt * k1);
    vec3 k3 = sampleVelocity(p + 0.5 * dt * k2);
    vec3 k4 = sampleVelocity(p + dt * k3);
    return p + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

void main() {
    uint id = gl_GlobalInvocationID.x;
    if (id >= particles.length()) return;

    Particle p = particles[id];
    float maxLife = max(0.1, p.vel_life.w);
    p.pos_age.w += uDeltaTime / maxLife;

    if (p.pos_age.w >= 1.0 || p.pos_age.x > uDomainMax.x || p.pos_age.x < uDomainMin.x ||
        p.pos_age.y > uDomainMax.y || p.pos_age.y < uDomainMin.y) {
        float r1 = hash(id + uSeed * 1999U);
        float r2 = hash(id * 31U + uSeed * 7919U);
        float r3 = hash(id * 73U + uSeed * 104729U);
        
        p.pos_age.x = uInletOrigin.x + (r1 - 0.5) * 2.0;
        p.pos_age.y = uInletOrigin.y + (r2 - 0.5) * uInletSize.x;
        p.pos_age.z = uInletOrigin.z + (r3 - 0.5) * uInletSize.y;
        p.pos_age.w = 0.0;
        p.vel_life.w = 2.0 + r1 * 4.0;
    } else {
        p.pos_age.xyz = rk4Step(p.pos_age.xyz, uDeltaTime);
        p.vel_life.xyz = sampleVelocity(p.pos_age.xyz);
    }

    particles[id] = p;
}
)";

GPUAdvection::GPUAdvection() = default;

GPUAdvection::~GPUAdvection() {
    if (initialized && context && surface) {
        context->makeCurrent(surface);
        if (velocityTexture) glDeleteTextures(1, &velocityTexture);
        if (ssbo) glDeleteBuffers(1, &ssbo);
        if (computeProgram) glDeleteProgram(computeProgram);
        context->doneCurrent();
    }
    if (surface) {
        surface->destroy();
        delete surface;
    }
    if (context) {
        delete context;
    }
}

bool GPUAdvection::initialize(int maxParticles) {
    if (!context) {
        context = new QOpenGLContext();
        QSurfaceFormat format;
        format.setVersion(4, 3);
        format.setProfile(QSurfaceFormat::CoreProfile);
        
        if (QOpenGLContext::globalShareContext()) {
            context->setShareContext(QOpenGLContext::globalShareContext());
        }
        
        context->setFormat(format);
        if (!context->create()) {
            std::cerr << "[GPU-Advection] Failed to create QOpenGLContext." << std::endl;
            return false;
        }
        
        surface = new QOffscreenSurface();
        surface->setFormat(context->format());
        surface->create();
    }
    
    context->makeCurrent(surface);

    if (!initializeOpenGLFunctions()) {
        std::cerr << "[GPU-Advection] Failed to initialize OpenGL 4.3 functions" << std::endl;
        return false;
    }

    numParticles = maxParticles;

    if (!initShaders()) {
        std::cerr << "[GPU-Advection] Failed to compile compute shader" << std::endl;
        return false;
    }

    initBuffers();
    initialized = true;
    std::cout << "[GPU-Advection] Initialized with " << numParticles << " particles on GPU." << std::endl;
    return true;
}

bool GPUAdvection::initShaders() {
    GLuint cShader = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(cShader, 1, &computeShaderSource, nullptr);
    glCompileShader(cShader);

    GLint success = 0;
    glGetShaderiv(cShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[1024];
        glGetShaderInfoLog(cShader, sizeof(infoLog), nullptr, infoLog);
        std::cerr << "[GPU-Advection] Shader Error: " << infoLog << std::endl;
        glDeleteShader(cShader);
        return false;
    }

    computeProgram = glCreateProgram();
    glAttachShader(computeProgram, cShader);
    glLinkProgram(computeProgram);

    glGetProgramiv(computeProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[1024];
        glGetProgramInfoLog(computeProgram, sizeof(infoLog), nullptr, infoLog);
        std::cerr << "[GPU-Advection] Program Link Error: " << infoLog << std::endl;
        glDeleteShader(cShader);
        glDeleteProgram(computeProgram);
        computeProgram = 0;
        return false;
    }

    glDeleteShader(cShader);
    return true;
}

void GPUAdvection::initBuffers() {
    glGenTextures(1, &velocityTexture);
    glBindTexture(GL_TEXTURE_3D, velocityTexture);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    std::vector<ParticleData> initialParticles(numParticles);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist01(0.0f, 1.0f);

    for (int i = 0; i < numParticles; ++i) {
        initialParticles[i].posX = domainMin[0] + dist01(rng) * (domainMax[0] - domainMin[0]);
        initialParticles[i].posY = domainMin[1] + dist01(rng) * (domainMax[1] - domainMin[1]);
        initialParticles[i].posZ = domainMin[2] + dist01(rng) * (domainMax[2] - domainMin[2]);
        initialParticles[i].age = dist01(rng);
        initialParticles[i].velX = 0.0f;
        initialParticles[i].velY = 0.0f;
        initialParticles[i].velZ = 0.0f;
        initialParticles[i].lifetime = 2.0f + dist01(rng) * 4.0f;
    }

    glGenBuffers(1, &ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, numParticles * sizeof(ParticleData), initialParticles.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void GPUAdvection::setDomainBounds(float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
    domainMin[0] = minX; domainMin[1] = minY; domainMin[2] = minZ;
    domainMax[0] = maxX; domainMax[1] = maxY; domainMax[2] = maxZ;
}

void GPUAdvection::setInletParams(float inletX, float centerY, float spanY, float spanZ) {
    inletOrigin[0] = inletX;
    inletOrigin[1] = centerY;
    inletOrigin[2] = 0.0f;
    inletSize[0] = spanY;
    inletSize[1] = spanZ;
}

void GPUAdvection::updateVelocityField(const float* vtkData, int nx, int ny, int nz) {
    if (!initialized || !vtkData || !context || !surface) return;
    context->makeCurrent(surface);

    int totalVoxels = nx * ny * nz;
    if ((int)texBuffer.size() < totalVoxels * 4) {
        texBuffer.resize(totalVoxels * 4);
    }

    for (int i = 0; i < totalVoxels; ++i) {
        texBuffer[i * 4 + 0] = vtkData[i * 3 + 0];
        texBuffer[i * 4 + 1] = vtkData[i * 3 + 1];
        texBuffer[i * 4 + 2] = vtkData[i * 3 + 2];
        texBuffer[i * 4 + 3] = 0.0f;
    }

    glBindTexture(GL_TEXTURE_3D, velocityTexture);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA32F, nx, ny, nz, 0, GL_RGBA, GL_FLOAT, texBuffer.data());
    glBindTexture(GL_TEXTURE_3D, 0);
}

void GPUAdvection::stepAdvection(float dt) {
    if (!initialized || !computeProgram || !context || !surface) return;
    context->makeCurrent(surface);

    frameSeed++;
    glUseProgram(computeProgram);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_3D, velocityTexture);
    glUniform1i(glGetUniformLocation(computeProgram, "uVelocityField"), 0);

    glUniform3fv(glGetUniformLocation(computeProgram, "uDomainMin"), 1, domainMin);
    glUniform3fv(glGetUniformLocation(computeProgram, "uDomainMax"), 1, domainMax);
    glUniform1f(glGetUniformLocation(computeProgram, "uDeltaTime"), dt);
    glUniform3fv(glGetUniformLocation(computeProgram, "uInletOrigin"), 1, inletOrigin);
    glUniform2fv(glGetUniformLocation(computeProgram, "uInletSize"), 1, inletSize);
    glUniform1ui(glGetUniformLocation(computeProgram, "uSeed"), frameSeed);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo);

    GLuint numGroups = (numParticles + 63) / 64;
    glDispatchCompute(numGroups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0);
    glUseProgram(0);
}

}
