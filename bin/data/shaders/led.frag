#version 120

uniform int u_ratio;

void main(){
    
	//this is the fragment shader
	//this is where the pixel level drawing happens
	//gl_FragCoord gives us the x and y of the current pixel its drawing
	
	//we grab the x and y and store them in an int
	int xVal = int(gl_FragCoord.x);
	int yVal = int(gl_FragCoord.y);
	
	//we use the mod function to only draw pixels if they are every 2 in x or every 4 in y
	if( mod(xVal, u_ratio) == 0 && mod(yVal, u_ratio) == 0 ){
		gl_FragColor = gl_Color;    
    }
    
}