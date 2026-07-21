#version 330 core

precision highp float;

in vec3 vclip;

uniform vec3 backgroundColor;
uniform vec3 ObjectColor;
uniform vec3 eyePosition;

uniform float shininess;
uniform float ambience;

uniform float fogBegin; // point at which fog begins
uniform float fogClamp; // point beyong which 100% fog is applied

out vec4 fragColor;

void main()
{
    vec3 lightColor = vec3(1.0, 0.95, 0.9);

    vec3 dist = eyePosition - vclip;

    vec3 dX = dFdx(vclip);
    vec3 dY = dFdy(vclip);

    vec3 normal = normalize(cross(dX, dY));
    vec3 lightDir = normalize(dist);

    float diffuseFactor = max(dot(normal, lightDir), 0.0);
    vec3 ambient = ambience * ObjectColor;
    vec3 diffuse = diffuseFactor * lightColor;

    vec3 viewDir = normalize(dist);
    vec3 reflectDir = reflect(-lightDir, normal);
    float specularFactor = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularFactor * lightColor * shininess;

    float fogFactor = clamp((fogClamp - length(dist)) / (fogClamp - fogBegin), 0.0, 1.0);

    vec3 colourOut = mix(backgroundColor, (ambient + diffuse + specular) * ObjectColor, fogFactor);

    fragColor = vec4(colourOut, 1.0);
}