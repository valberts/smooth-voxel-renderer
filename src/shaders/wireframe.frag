#version 330 core

in vec3 fragWorldPos;
out vec4 FragColor;

uniform vec3 wireframeColor;
uniform vec3 cameraPos;
uniform bool useFading;
uniform float fadeStart;
uniform float fadeEnd;

void main() {
    float alpha = 1.0;
    
    if (useFading) {
        float distance = length(fragWorldPos - cameraPos);
        
        alpha = 1.0 - smoothstep(fadeStart, fadeEnd, distance);
        
        if (alpha < 0.01) {
            discard;
        }
    }
    
    FragColor = vec4(wireframeColor, alpha);
}
