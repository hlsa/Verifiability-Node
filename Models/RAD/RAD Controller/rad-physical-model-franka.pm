import physmod::joints::Revolute
import physmod::actuators::*
import physmod::math::*

sensor ForceSensor {
	// Very basic force sensor; needs to be replaced with something more sophisticated
	input input_force: real
	output output_force: real
	equation input_force == output_force
}

sensor TorqueSensor {
	// Very basic force sensor; needs to be replaced with something more sophisticated
	input input_torque: real
	output output_torque: real
	equation input_torque == output_torque
}

pmodel pmfranka {
	const PI: real = 3.1415	
	const ARM_RADIUS: real = 0.055
	const GRIP_RADIUS: real = 0.02
	const GRIP_LENGTH: real = 0.08
	const Z_BASE: real = 0
	const X_BASE: real = 0
	const LEN1: real = 0.3330
	const LEN2: real = 0.3160
	const LEN3: real = 0.3840
	const LEN4: real = 0.0825
	const LEN5: real = 0.0880
	const LEN6: real = 0.1070
	// const masses: vector(real, 7) = [|3.4525, 3.4821, 4.0562, 3.4822, 2.1633, 2.3466, 0.31290|]
	const theta0: real = PI/2
	const alpha0: real = PI/2	
	
	local link Li0 {
		def {
			inertial information {
				mass 3.4525 kg
			}
		}
		local body Cy0 {
			def {
				cylinder ( radius = ARM_RADIUS , length = LEN1 )
			}
			pose {
				x = 0.0
				y = 0.0
				z = LEN1/2
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		jref J0 = Revolute {
			instantiation AXIS = (|0,0,1|)
			aref M0 = SpeedControlMotor {
				relation J0.tau == M0.T
			}
			flexibly connected to Li1
		}
		sref FSJ0 = ForceSensor
		pose {
			x = X_BASE
			y = 0.0
			z = Z_BASE
			roll = 0.0
			pitch = 0.0
			yaw = 0.0
		}
	}
	local link Li1 {
		def {
		inertial information {
				mass 3.4821 kg
			}
		}
		local body Cy1 {
			def {
				cylinder ( radius = ARM_RADIUS , length = 2*ARM_RADIUS )
			}
			pose {
				x = 0.0
				y = 0.0
				z = 0.0
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		local body Sp1a {
			def {
				sphere ( radius = ARM_RADIUS)
			}
			pose {
				x = 0.0
				y = 0.0
				z = ARM_RADIUS
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		local body Sp1b {
			def {
				sphere ( radius = ARM_RADIUS)
			}
			pose {
				x = 0.0
				y = 0.0
				z = -ARM_RADIUS
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		jref J1 = Revolute {
			instantiation AXIS = (|0,0,1|)
			aref MS1 = SpeedControlMotor {
				relation J1.tau == MS1.T
			}
			flexibly connected to Li2
		}
		sref FSJ1 = ForceSensor
		pose {
			x = 0.0
			y = 0.0
			z = 0.333
			roll = PI/2
			pitch = 0.0
			yaw = 0.0
		}
	}
	local link Li2 {
		def {
		inertial information {
				mass 3.4821 kg
			}
		}
		local body Cy2 {
			def {
				cylinder ( radius = ARM_RADIUS , length = LEN2 )
			}
			pose {
				x = 0.0
				y = 0.0
				z = LEN2/2
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		jref J2 = Revolute {
			instantiation AXIS = (|0,0,1|)
			aref M2 = SpeedControlMotor {
				relation J2.tau == M2.T
			}
			flexibly connected to Li3
		}
		sref FSJ2 = ForceSensor
		pose {
			x = 0.0
			y = 0.0
			z = 0.333
			roll = 0.0
			pitch = 0.0
			yaw = 0.0
		}
	}
	local link Li3 {
		def {
		inertial information {
				mass 4.0562 kg
			}
		}
		local body Cy3 {
			def {
				cylinder ( radius = ARM_RADIUS , length = 2*ARM_RADIUS )
			}
			pose {
				x = 0.0
				y = 0.0
				z = 0.0
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		local body Sp3a {
			def {
				sphere ( radius = ARM_RADIUS)
			}
			pose {
				x = 0.0
				y = 0.0
				z = ARM_RADIUS
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		local body Sp3b {
			def {
				sphere ( radius = ARM_RADIUS)
			}
			pose {
				x = 0.0
				y = 0.0
				z = -ARM_RADIUS
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		jref J3 = Revolute {
			instantiation AXIS = (|0,0,1|)
			aref M3 = SpeedControlMotor {
				relation J3.tau == M3.T
			}
			flexibly connected to Li4
		}
		sref FSJ3 = ForceSensor
		pose {
			x = LEN4
			y = 0.0
			z = 0.649
			roll = PI/2
			pitch = 0.0
			yaw = 0.0
		}
	}
	local link Li4 {
		def {
		inertial information {
				mass 3.4822 kg
			}
		}
		local body Cy4 {
			def {
				cylinder ( radius = ARM_RADIUS , length = LEN3 )
			}
			pose {
				x = 0.0
				y = 0.0
				z = LEN3/2
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		jref J4 = Revolute {
			instantiation AXIS = (|0,0,1|)
			aref M4 = SpeedControlMotor {
				relation J4.tau == M4.T
			}
			flexibly connected to Li5
		}
		sref FSJ4 = ForceSensor
		pose {
			x = 0.0
			y = 0.0
			z = 0.649
			roll = 0.0
			pitch = 0.0
			yaw = 0.0
		}
	}
	local link Li5 {
		def {
		inertial information {
				mass 2.1633 kg
			}
		}
		local body Cy5 {
			def {
				cylinder ( radius = ARM_RADIUS , length = 2*ARM_RADIUS )
			}
			pose {
				x = 0.0
				y = 0.0
				z = 0.0
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		jref J5 = Revolute {
			instantiation AXIS = (|0,0,1|)
			aref M5 = SpeedControlMotor {
				relation J5.tau == M5.T
			}
			flexibly connected to Li6
		}
		sref FSJ5 = ForceSensor
		pose {
			x = 0.0
			y = 0.0
			z = LEN1 + LEN2 + LEN3 
			roll = PI/2
			pitch = 0.0
			yaw = 0.0
		}
	}
	local link Li6 {
		def {
		inertial information {
				mass 2.3466 kg
			}
		}
		local body Cy6 {
			def {
				cylinder ( radius = ARM_RADIUS , length = LEN5 )
			}
			pose {
				x = 0.0
				y = 0.0
				z = LEN5/2
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		jref J6 = Revolute {
			instantiation AXIS = (|0,1,0|)
			aref M6 = SpeedControlMotor {
				relation J6.tau == M6.T
			}
			flexibly connected to Li7
		}
		sref FSJ6 = ForceSensor
		pose {
			x = 0.0
			y = 0.0
			z = LEN1 + LEN2 + LEN3
			roll = PI/2
			pitch = 0.0
			yaw = PI/2
		}
	}
	local link Li7 {
		def {
		inertial information {
				mass 0.3129 kg
			}
		}
		local body Sp7 {
			def {
				sphere ( radius = ARM_RADIUS )
			}
			pose {
				x = 0.0
				y = 0.0
				z = 0.0
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		local body Cy7 {
			def {
				cylinder ( radius = ARM_RADIUS , length = LEN6 )
			}
			pose {
				x = 0.0
				y = 0.0
				z = LEN6/2
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		jref J7 = Revolute {
			instantiation AXIS = (|0,1,0|)
			aref M7 = SpeedControlMotor {
				relation J7.tau == M7.T
			}
			flexibly connected to LiF
		}
		sref FSJ7 = ForceSensor
		pose {
			x = LEN5
			y = 0.0
			z = LEN1 + LEN2 + LEN3
			roll = 0.0
			pitch = -PI
			yaw = 0.0
		}
	}
	local link LiF {
		def {
		}
		local body CyF {
			def {
				cylinder ( radius = GRIP_RADIUS , length = GRIP_LENGTH )
			}
			pose {
				x = 0.0
				y = 0.0
				z = GRIP_LENGTH/2
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		pose {
			x = LEN5
			y = 0.0
			z = LEN1 + LEN2 + LEN3 - LEN6
			roll = 0.0
			pitch = -PI
			yaw = 0.0
		}
		sref FSG = ForceSensor
		aref GR1 = Gripper
	}
}
sensor NoiseSensor {
	output output_noise_level: real
}
actuator Gripper {
}
