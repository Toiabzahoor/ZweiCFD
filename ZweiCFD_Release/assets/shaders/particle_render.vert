#version 430
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
    
    float trailLength = min(len * 0.5, 0.2);
    vec3 trailEnd = pt.pos - dir * max(trailLength, 0.05); 
    
    // World space billboard normal
    vec3 up = vec3(0.0, 0.0, 1.0);
    if (abs(dir.z) > 0.99) up = vec3(0.0, 1.0, 0.0);
    vec3 normal = normalize(cross(dir, up));
    
    vec3 finalPos3D = (vertexTexCoord.x > 0.5) ? pt.pos : trailEnd;
    
    float thickness = max(0.005, pt.baseSize * 1.5);
    if (vertexTexCoord.y > 0.5) finalPos3D += normal * thickness;
    else finalPos3D -= normal * thickness;
    
    // Check for NaN to protect GPU
    if (isnan(finalPos3D.x) || isnan(finalPos3D.y) || isnan(finalPos3D.z)) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        return;
    }
    
    gl_Position = mvp * vec4(finalPos3D, 1.0);
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
    
    // Bernoulli opacity modulation: fade out particles that haven't changed velocity
    float speedRatio = pt.velocityMag / max(0.001, v_inf);
    float speedDiff = abs(speedRatio - 1.0);
    float bernoulliFade = mix(0.4, 1.0, smoothstep(0.0, 0.1, speedDiff));
    
    float finalAlpha = pt.alpha * fadeIn * bernoulliFade;
    
    fragColor = vec4(baseColor, finalAlpha);
    fragTexCoord = vertexTexCoord;
}
