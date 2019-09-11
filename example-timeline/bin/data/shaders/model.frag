#define PI (3.1415926536)
#define TWO_PI (6.2831853072)

#ifdef GL_ES
precision mediump float;
#endif

uniform float elapsedTime;
uniform float stage;
uniform float alpha;
varying vec3 position, normal;
varying float randomOffset;

//float time = elapsedTime;

const vec4 on = vec4(1.);
const vec4 g  = vec4(0,1,0,1);
const vec4 off = vec4(vec3(0.), 1.);

void main() {


    vec3 col = vec3(vec3(0.7)*dot(abs(normal), vec3(0.7)));
    gl_FragColor = vec4(col, 1.);
        
	
}
