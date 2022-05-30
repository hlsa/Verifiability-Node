import physmod::joints::Revolute
import physmod::actuators::*

pmodel pmbaxter4 {
	const PI: real = 3.1415	
	const ARM_RADIUS: real = 0.069
	const LENGTH_S: real = 0.27035
	const LENGTH_E: real = 0.36435
	const LENGTH_WA: real = 0.37429
	const LENGTH_WB: real = 0.22953
	const LENGTH_GRIPPER: real = 0.17
	const GRIPPER_RADIUS: real = 0.022
	const DISPLACEMENT1: real = 0.069
	const DISPLACEMENT2: real = 0.069
	const DISPLACEMENT3: real = 0.01
	const Z_BASE: real = 1.0345
	const X_BASE: real = 0.4
	
	local link LiBase {
		def {}
		local body CyB0 {
			def {
				cylinder ( radius = 0.2 , length = 0.9 )
			}
			pose {
				x = 0
				y = 0
				z = 0.45
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		local body CyB1 {
			def {
				cylinder ( radius = ARM_RADIUS , length = 0.471 )
			}
			pose {
				x = 0.2
				y = 0
				z = 0.95
				roll = 0
				pitch = -PI/2
				yaw = 0.0
			}
		}
		local body CyB2 {
			def {
				cylinder ( radius = ARM_RADIUS , length = 0.1 )
			}
			pose {
				x = 0.4
				y = 0
				z = 0.9845
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		local body SpB {
			def {
				sphere ( radius = 0.2 )
			}
			pose {
				x = 0
				y = 0
				z = 0.85
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
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
	local link LiS0 {
		def {
		}
		local body CyS {
			def {
				cylinder ( radius = ARM_RADIUS , length = LENGTH_S )
			}
			pose {
				x = 0.0
				y = 0.0
				z = LENGTH_S/2
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		jref JS0 = Revolute {
			instantiation AXIS = (|0,1,0|)
			aref MS0 = SpeedControlMotor {
				relation JS0.tau == MS0.T
			}
			flexibly connected to LiS1
		}
		pose {
			x = X_BASE
			y = 0.0
			z = Z_BASE
			roll = 0.0
			pitch = 0.0
			yaw = 0.0
		}
	}
	local link LiS1 {
		def {}
		jref JS1 = Revolute {
			instantiation AXIS = (|0,0,1|)
			aref MS1 = SpeedControlMotor {
				relation JS1.tau == MS1.T
			}
			flexibly connected to LiE0
		}
		pose {
			x = X_BASE + DISPLACEMENT1
			y = 0
			z = Z_BASE + LENGTH_S
			roll = 0.0
			pitch = 0.0
			yaw = 0.0
		}
	}
	local link LiE0 {
		def {}
		jref JE0 = Revolute {
			instantiation AXIS = (|0,0,1|)
			aref ME0 = SpeedControlMotor {
				relation JE0.tau == ME0.T
			}
			flexibly connected to LiE1
		}
		pose {
			x = X_BASE + DISPLACEMENT1
			y = 0.0
			z = Z_BASE + LENGTH_S
			roll = 0.0
			pitch = 0.0
			yaw = 0.0
		}
		local body CyE {
			def {
				cylinder ( radius = ARM_RADIUS , length = LENGTH_E )
			}
			pose {
				x = 0.0
				y = 0.0
				z = LENGTH_E/2
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
	}
	local link LiE1 {
		def {}
		jref JE1 = Revolute {
			instantiation AXIS = (|0,1,0|)
			aref ME1 = SpeedControlMotor {
				relation JE1.tau == ME1.T
			}
			flexibly connected to LiW0
		}
		pose {
			x = X_BASE + DISPLACEMENT1 + DISPLACEMENT2
			y = 0.0
			z = Z_BASE + LENGTH_S + LENGTH_E
			roll = 0.0
			pitch = PI/2
			yaw = 0.0
		}
	}
	local link LiW0 {
		def {}
		jref JW0 = Revolute {
			instantiation AXIS = (|0,0,1|)
			aref MW0 = SpeedControlMotor {
				relation JW0.tau == MW0.T
			}
			flexibly connected to LiW1
		}
		pose {
			x = X_BASE + DISPLACEMENT1 + DISPLACEMENT2
			y = 0.0
			z = Z_BASE + LENGTH_S + LENGTH_E
			roll = 0.0
			pitch = PI/2
			yaw = 0.0
		}
		local body CyWA {
			def {
				cylinder ( radius = ARM_RADIUS , length = LENGTH_WA )
			}
			pose {
				x = 0.0
				y = 0.0
				z = LENGTH_WA/2
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
	}
	local link LiW1 {
		def {}
		jref JW1 = Revolute {
			instantiation AXIS = (|0,1,0|)
			aref MW1 = SpeedControlMotor {
				relation JW1.tau == MW1.T
			}
			flexibly connected to LiW2
		}
		pose {
			x = X_BASE + DISPLACEMENT1 + DISPLACEMENT2 + LENGTH_WA
			y = 0.0
			z = Z_BASE + LENGTH_S + LENGTH_E + DISPLACEMENT3
			roll = 0.0
			pitch = PI/2
			yaw = 0.0
		}
		local body CyWB {
			def {
				cylinder ( radius = ARM_RADIUS , length = LENGTH_WB )
			}
			pose {
				x = 0.0
				y = 0.0
				z = LENGTH_WB/2
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
	}
	local link LiW2 {
		def {}
		pose {
			x = X_BASE + DISPLACEMENT1 + DISPLACEMENT2 + LENGTH_WA + LENGTH_WB
			y = 0.0
			z = Z_BASE + LENGTH_S + LENGTH_E + DISPLACEMENT3
			roll = 0.0
			pitch = PI/2
			yaw = 0.0
		}
		local body CyEE {
			def {
				cylinder ( radius = GRIPPER_RADIUS , length = LENGTH_GRIPPER )
			}
			pose {
				x = 0.0
				y = 0.0
				z = LENGTH_GRIPPER/2
				roll = 0.0
				pitch = 0.0
				yaw = 0.0
			}
		}
		local body SpEE {
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
	}
}
