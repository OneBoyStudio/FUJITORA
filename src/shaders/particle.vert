#version 330 core

layout (location = 0) in vec3 aPosition;
layout (location = 1) in float aLife;

uniform mat4 Projection;
uniform mat4 View;

out float vLife;

void main()
{
    vLife = aLife;

    float maxSize = 60.0;
    float minSize = 20.0;
    gl_PointSize = mix(minSize, maxSize, 1.0 - aLife);

    gl_Position = Projection * View * vec4(aPosition, 1.0);
}