#version 330 core

in float vLife;
out vec4 FragColor;

void main() {
    vec2 center = gl_PointCoord - vec2(0.5);
    float dist = length(center);

    if (dist > 0.5 || (vLife <= 0.0) ) {
        discard;
    }

    vec3 hotColor = vec3(1.0, 0.9, 0.5);
    vec3 coolColor = vec3(0.9, 0.3, 0.02);
    vec3 finalColor = mix(coolColor, hotColor, vLife);

    if (vLife == 0) {
        finalColor = vec3(0.0, 0.0, 0.0);
    }

    float alpha = (1.0 - (dist * 2.0)) * vLife;

    FragColor = vec4(finalColor, alpha);
}