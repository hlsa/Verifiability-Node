pmodel RotorArm {
	const ARM_LENGTH : real = 25.99 cm
	const ARM_RADIUS : real = 1.4 cm
	local link arm {
		def {
			
		}
		pose {
			x = 0
			y = 0
			z = 0
			roll = 0
			pitch = 0
			yaw = 0
		}
	local body Pole {
			def {
				cylinder ( radius = ARM_RADIUS , length = ARM_LENGTH )
				inertial information {
					mass 0.25kg
				}
			}
			pose {
				x = 
				( ARM_LENGTH / 2 )
				y = 0
				z = 0
				roll = 0
				pitch = - PI / 2
				yaw = 0
			}
		}
	local body MotorHolder {
			def {
				cylinder ( radius = MOTOR_HOLDER_RADIUS , length = ARM_RADIUS*2 )
				inertial information {
					mass 0.05kg
				}
			}
			pose {
				x = ARM_LENGTH+MOTOR_HOLDER_RADIUS
				y = 0
				z = 0
				roll = 0
				pitch = 0
				yaw = 0
			}
		}
	}
	const MOTOR_HOLDER_RADIUS : real = 3.6 cm
}
