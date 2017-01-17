#define PI (3.1415926536)
#define TWO_PI (6.2831853072)

#ifdef GL_ES
precision mediump float;
#endif
varying vec3 position, normal;

void main() {
    vec3 col = normal*dot(normal,vec3(0.0, 1.0, 1.0));
    gl_FragColor = vec4(col, 1.);
    
}
