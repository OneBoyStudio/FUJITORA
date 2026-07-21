#version 330 core

in vec3 aPosition;

uniform mat4 Projection;
uniform mat4 View;
uniform mat4 Model;

out vec3 vclip;

void main()
{
    vec4 clip = Model * vec4(aPosition, 1.0);
    vclip = clip.xyz;

    gl_Position = Projection * View * clip;
}