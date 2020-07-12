function xdot=edsonjPap(u1,u2,u3,u4,u5)
// This is nonlinear Pendulum Model

// Load the parameters
exec('edsonjParametersPap.sce', -1);

// state variables
x=u1;		
v=u2;
theta=u3;
vartheta=u4;

// control variables
F=u5;	// fuerza


// Modelo MoDiCA-X
//Estado 1-posicion
e1dot= v;
//Estado 2-Velocidad lineal respecto a 1
e2dot= ((F*(I+m*l^2))/((M+m)*(I+m*l^2)-(m^2*l^2*(cos(theta))^2)))-...
        ((v*b*(I+m*l^2))/((M+m)*(I+m*l^2)-(m^2*l^2*(cos(theta))^2)))+...
((m^2*l^2*g*sin(theta)*cos(theta))/((M+m)*(I+m*l^2)-(m^2*l^2*(cos(theta))^2)))+...
((m*l*vartheta^2*sin(theta)*(I+m*l^2))/((M+m)*(I+m*l^2)-(m^2*l^2*(cos(theta))^2)));
//Estado 3-Angulo theta
e3dot= vartheta;
//Estado 4-Velocidad Angular respecto de 3
e4dot= -((m*l*cos(theta)*F)/((M+m)*(I+m*l^2)-(m^2*l^2*(cos(theta))^2)))+...
        ((v*b*m*l*cos(theta))/((M+m)*(I+m*l^2)-(m^2*l^2*(cos(theta))^2)))-...
        ((m^2*l^2*vartheta^2*sin(theta)*cos(theta))/((M+m)*(I+m*l^2)-(m^2*l^2*(cos(theta))^2)))-...
        ((m*g*l*(M+m)*sin(theta))/((M+m)*(I+m*l^2)-(m^2*l^2*(cos(theta))^2)));

//Salida xdot
xdot =[e1dot;e2dot;e3dot;e4dot]; 
endfunction
