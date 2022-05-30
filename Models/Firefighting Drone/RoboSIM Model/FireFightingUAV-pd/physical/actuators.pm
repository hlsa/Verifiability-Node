
// Pump/Nozzle
actuator Nozzle {
	
	// Receives a pressure input?
	input P:real
	
	// Area of the nozzle section?
	const A:real
	
	// It produces a force on the water coming out of the nozzle.
	output F:real
	
	equation P == F/A
}

actuator ServoMotor {
	
	input dangle: real 
	output T: real
	local Tm: real, Vemf: real, Tf: real 
	local V: real, i: real
	local theta: real, av: real, e: real
	
	const b: real, Ke: real, Kt: real
	const R: real, L: real
	const Kp: real, Ki: real, Kd: real
	
	equation av == derivative(theta)
	equation Tm == Kt*i
	equation Vemf == Ke*av
	equation Tf == b*av
	equation T == Tm - Tf
	equation V == i*R+L*derivative(i)+Vemf 
	equation e == dangle-theta //das - av
	equation V == Kp*e+Ki*integral(e,0,t)+Kd*derivative(e)
}

actuator Comms {
	input inp:string
	output oup:string
	equation oup==inp
}