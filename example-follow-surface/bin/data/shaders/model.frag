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
//	float stage = 4.;
	
	if(stage < 1.) {

        vec3 col = (vec3(0.5)/abs(normal))*dot(vec3(1., 0, 1.), normal)+vec3(0.75, 0, 0.75);
		gl_FragColor = vec4(col, 1.);
        
	} else if(stage < 2.) {
        gl_FragColor = vec4(sin(mod(position.x + elapsedTime*2., PI)),
                            sin(mod(-randomOffset + elapsedTime*2., TWO_PI)),
                            0,
                            1);
	} else if(stage < 3.) {
		// crazy triangles, grid lines
		float speed = 1.;
		float scale = 10.0;
		float cutoff = .7;
		vec3 cur = mod(position + speed * elapsedTime, scale) / scale;
		cur *= 1.-abs(normal)/2.;
		gl_FragColor = (max(max(cur.x, cur.y), cur.z) < cutoff) ? off : vec4(sin(mod(position.y + elapsedTime, PI)),
                                                                             sin(mod(position.x + elapsedTime, TWO_PI)),
                                                                             sin(mod(position.z+elapsedTime,PI)),
                                                                             1);
	} else if(stage < 4.) {
        vec3 col = vec3(1.)*normal*dot(normal, vec3(1.));
        gl_FragColor = vec4(col, 1.);
    	
    } else if (stage < 5.) {
    	gl_FragColor = vec4(normal * 0.5 + 0.5, alpha);
    } else if (stage < 7.) {
    	float t = mod(abs(sin(elapsedTime/60.))*120., abs(cos(normal.y/normal.x))*42.);
    	float d = mod(dot(position, abs(normal))/abs(normal.x) * abs(cos(elapsedTime*20.))*1.55, t) / (t);
    	float a = sin(d * TWO_PI) * 0.5 + 0.5;
        vec3 col = (vec3(0.5)/abs(normal))*dot(vec3(1., 0, 1.), normal)+vec3(0.75, 0, 0.75);
        //if(sin(dot(position.z, normal.z)/position.y) > 0.0){
        if (a > abs(sin(elapsedTime/60.))*.125){
            gl_FragColor = vec4(col, 1.);
        }else{
            discard;
        }
//                }else{
        //            gl_FragColor = vec4(0, 0, 0, 1.);
        //        }
    }else if(stage< 8.){
        float t = max(mod((abs(sin(elapsedTime*0.1270)))*100.0, 564.0), 9.0);
    	float d = mod(position.y * 1.0, t) / t;
    	float a = sin(d * TWO_PI) * 0.5 + 0.50;
    	if (a > 0.5) gl_FragColor =vec4(1, 0, 1, 1);
	    else gl_FragColor = vec4(vec3(0, 0, 0), 1);
    }else if(stage<9.){
        float t = max(mod((abs(sin(elapsedTime*0.1270)))*100.0, 564.0), 9.0);
    	float d = mod(position.x * 1.0, t) / t;
    	float a = sin(d * TWO_PI) * 0.5 + 0.50;
    	if (a > 0.5) gl_FragColor =vec4(sin(mod(position.y + elapsedTime, PI)),
                                        sin(mod(position.x + elapsedTime, TWO_PI)),
                                        sin(mod(position.z+elapsedTime,PI)),
                                        1);
        else discard;
    }
}
