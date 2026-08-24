#pragma once

#include <vector>
#include <memory>
#ifndef __APPLE__
#include <QOpenGLFunctions_4_3_Core>
#else
#include <QOpenGLFunctions>
#endif
#include <QOpenGLContext>
#include <QOffscreenSurface>

namespace zweicfd {

struct alignas(16) ParticleData {
    float posX, posY, posZ, age;
    float velX, velY, velZ, lifetime;
};

#ifndef __APPLE__
class GPUAdvection : protected QOpenGLFunctions_4_3_Core {
#else
class GPUAdvection : protected QOpenGLFunctions {
#endif
public:
    GPUAdvection();
    ~GPUAdvection();

    bool initialize(int maxParticles = 65536);
    void updateVelocityField(const float* vtkData, int nx, int ny, int nz);
    void setDomainBounds(float minX, float minY, float minZ, float maxX, float maxY, float maxZ);
    void setInletParams(float inletX, float centerY, float spanY, float spanZ);
    void stepAdvection(float dt);
    
    GLuint getParticleSSBO() const { return ssbo; }
    int getParticleCount() const { return numParticles; }
    bool isReady() const { return initialized; }

private:
    bool initialized = false;
    int numParticles = 65536;
    GLuint velocityTexture = 0;
    GLuint ssbo = 0;
    GLuint computeProgram = 0;
    unsigned int frameSeed = 0;
    
    float domainMin[3] = {-50.0f, -25.0f, -6.0f};
    float domainMax[3] = {50.0f, 25.0f, 6.0f};
    float inletOrigin[3] = {-45.0f, 0.0f, 0.0f};
    float inletSize[2] = {20.0f, 10.0f};
    
    std::vector<float> texBuffer;
    
    QOpenGLContext* context = nullptr;
    QOffscreenSurface* surface = nullptr;
    
    bool initShaders();
    void initBuffers();
};

}
