varying vec3 normal;
uniform vec4 color;

void main() {

    vec3 col = vec3(vec3(0.7)*dot(abs(normal), vec3(0.7)));
    gl_FragColor = vec4(col, 1.)*color;
    
}
