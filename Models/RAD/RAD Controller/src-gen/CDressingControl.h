#ifndef ROBOCALC_CONTROLLERS_CDRESSINGCONTROL_H_
#define ROBOCALC_CONTROLLERS_CDRESSINGCONTROL_H_

#include "Franka.h"
#include "RoboCalcAPI/Controller.h"
#include "DataTypes.h"

#include "SDressingControl.h"
#include "MovementControl.h"
#include "EmergencyStopControl.h"
#include "EnvironmentalMonitor.h"

class CDressingControl: public robocalc::Controller 
{
public:
	CDressingControl(Franka& _platform) : platform(&_platform){};
	CDressingControl() : platform(nullptr){};
	
	~CDressingControl() = default;
	
	void Execute()
	{
		sDressingControl.execute();
		movementControl.execute();
		emergencyStopControl.execute();
		environmentalMonitor.execute();
	}
	
	struct Channels
	{
		CDressingControl& instance;
		Channels(CDressingControl& _instance) : instance(_instance) {}
		
		EventBuffer* tryEmitSnaggingDetected(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceiveSnaggingDetected(args))
				instance.sDressingControl.snaggingDetected_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitInvalidTrajectory(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceiveInvalidTrajectory(args))
				instance.sDressingControl.invalidTrajectory_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitInvalidPoseDetected(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceiveInvalidPoseDetected(args))
				instance.sDressingControl.invalidPoseDetected_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitUserInterrupt(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceiveUserInterrupt(args))
				instance.sDressingControl.userInterrupt_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitPoseDetected(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceivePoseDetected(args))
				instance.sDressingControl.poseDetected_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitDressingRequest(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceiveDressingRequest(args))
				instance.sDressingControl.dressingRequest_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitGarmentDetected(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceiveGarmentDetected(args))
				instance.sDressingControl.garmentDetected_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitEmergencyStop(void* sender, std::tuple<> args)
		{
			if(instance.emergencyStopControl.canReceiveEmergencyStop(args))
				instance.emergencyStopControl.emergencyStop_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitResumeDressing(void* sender, std::tuple<> args)
		{
			if(instance.emergencyStopControl.canReceiveResumeDressing(args))
				instance.emergencyStopControl.resumeDressing_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitEnvironmentalResume(void* sender, std::tuple<> args)
		{
			if(instance.emergencyStopControl.canReceiveEnvironmentalResume(args))
				instance.emergencyStopControl.environmentalResume_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitEnvironmentalStop(void* sender, std::tuple<> args)
		{
			if(instance.emergencyStopControl.canReceiveEnvironmentalStop(args))
				instance.emergencyStopControl.environmentalStop_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitBackgroundNoiseLevel(void* sender, std::tuple<NoiseLevel> args)
		{
			if(instance.environmentalMonitor.canReceiveBackgroundNoiseLevel(args))
				instance.environmentalMonitor.backgroundNoiseLevel_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitOtherAgentDetected(void* sender, std::tuple<bool> args)
		{
			if(instance.environmentalMonitor.canReceiveOtherAgentDetected(args))
				instance.environmentalMonitor.otherAgentDetected_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitDressedJoint(void* sender, std::tuple<int> args)
		{
			dressedJoint_in.trigger(sender, args);
			return nullptr;
		}
		
		EventBuffer* tryEmitAllowMovement(void* sender, std::tuple<> args)
		{
			if(instance.movementControl.canReceiveAllowMovement(args))
				instance.movementControl.allowMovement_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitMovementEnd(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceiveMovementEnd(args))
				instance.sDressingControl.movementEnd_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitArmConfig(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceiveArmConfig(args))
				instance.sDressingControl.armConfig_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitMovementStart(void* sender, std::tuple<> args)
		{
			if(instance.movementControl.canReceiveMovementStart(args))
				instance.movementControl.movementStart_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitGripCorrect(void* sender, std::tuple<bool> args)
		{
			if(instance.sDressingControl.canReceiveGripCorrect(args))
				instance.sDressingControl.gripCorrect_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitSecondHandDetected(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceiveSecondHandDetected(args))
				instance.sDressingControl.secondHandDetected_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitEndEffectorPosition(void* sender, std::tuple<> args)
		{
			if(instance.movementControl.canReceiveEndEffectorPosition(args))
				instance.movementControl.endEffectorPosition_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitUserDressed(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceiveUserDressed(args))
				instance.sDressingControl.userDressed_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitCorrectedMovement(void* sender, std::tuple<> args)
		{
			correctedMovement_in.trigger(sender, args);
			return nullptr;
		}
		
		EventBuffer* tryEmitGripperEngaged(void* sender, std::tuple<bool> args)
		{
			if(instance.sDressingControl.canReceiveGripperEngaged(args))
				instance.sDressingControl.gripperEngaged_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitForceEndEffector(void* sender, std::tuple<> args)
		{
			if(instance.movementControl.canReceiveForceEndEffector(args))
				instance.movementControl.forceEndEffector_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitForceStart(void* sender, std::tuple<> args)
		{
			if(instance.movementControl.canReceiveForceStart(args))
				instance.movementControl.forceStart_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitForceEnd(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceiveForceEnd(args))
				instance.sDressingControl.forceEnd_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitCollisionStop(void* sender, std::tuple<> args)
		{
			if(instance.emergencyStopControl.canReceiveCollisionStop(args))
				instance.emergencyStopControl.collisionStop_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitCollisionResume(void* sender, std::tuple<> args)
		{
			if(instance.emergencyStopControl.canReceiveCollisionResume(args))
				instance.emergencyStopControl.collisionResume_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitMovementOccurred(void* sender, std::tuple<> args)
		{
			if(instance.emergencyStopControl.canReceiveMovementOccurred(args))
				instance.emergencyStopControl.movementOccurred_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitMovementInitiated(void* sender, std::tuple<> args)
		{
			movementInitiated_in.trigger(sender, args);
			return nullptr;
		}
		
		EventBuffer* tryEmitStepMoved(void* sender, std::tuple<> args)
		{
			stepMoved_in.trigger(sender, args);
			return nullptr;
		}
		
		EventBuffer* tryEmitMovementWasAllowed(void* sender, std::tuple<> args)
		{
			movementWasAllowed_in.trigger(sender, args);
			return nullptr;
		}
		
		EventBuffer* tryEmitAllowedMovement(void* sender, std::tuple<> args)
		{
			allowedMovement_in.trigger(sender, args);
			return nullptr;
		}
		
		EventBuffer* tryEmitNoticedMovementOccurred(void* sender, std::tuple<> args)
		{
			noticedMovementOccurred_in.trigger(sender, args);
			return nullptr;
		}
		
		EventBuffer* tryEmitForceWasAllowed(void* sender, std::tuple<> args)
		{
			forceWasAllowed_in.trigger(sender, args);
			return nullptr;
		}
		
		EventBuffer* tryEmitForceInitiated(void* sender, std::tuple<> args)
		{
			forceInitiated_in.trigger(sender, args);
			return nullptr;
		}
		
		EventBuffer* tryEmitResetMovement(void* sender, std::tuple<> args)
		{
			if(instance.movementControl.canReceiveResetMovement(args))
				instance.movementControl.resetMovement_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitForceApplied(void* sender, std::tuple<> args)
		{
			forceApplied_in.trigger(sender, args);
			return nullptr;
		}
		
		EventBuffer* tryEmitUserMoved(void* sender, std::tuple<> args)
		{
			if(instance.sDressingControl.canReceiveUserMoved(args))
				instance.sDressingControl.userMoved_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitHandlingCorrect(void* sender, std::tuple<bool> args)
		{
			if(instance.sDressingControl.canReceiveHandlingCorrect(args))
				instance.sDressingControl.handlingCorrect_in.trigger(sender, args);
				
			return nullptr;
		}
		
		EventBuffer* tryEmitGarmentAtHand(void* sender, std::tuple<> args)
		{
			garmentAtHand_in.trigger(sender, args);
			return nullptr;
		}
		
		EventBuffer* tryEmitArrivedAtGarment(void* sender, std::tuple<> args)
		{
			arrivedAtGarment_in.trigger(sender, args);
			return nullptr;
		}
		
		EventBuffer* tryEmitGarmentGripped(void* sender, std::tuple<> args)
		{
			garmentGripped_in.trigger(sender, args);
			return nullptr;
		}
		
		struct DressedJoint_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<int> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<int> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_args = args;
				_sender = sender;
			}
		} dressedJoint_in;
		struct CorrectedMovement_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} correctedMovement_in;
		struct MovementInitiated_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} movementInitiated_in;
		struct StepMoved_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} stepMoved_in;
		struct MovementWasAllowed_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} movementWasAllowed_in;
		struct AllowedMovement_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} allowedMovement_in;
		struct NoticedMovementOccurred_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} noticedMovementOccurred_in;
		struct ForceWasAllowed_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} forceWasAllowed_in;
		struct ForceInitiated_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} forceInitiated_in;
		struct ForceApplied_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} forceApplied_in;
		struct GarmentAtHand_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} garmentAtHand_in;
		struct ArrivedAtGarment_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} arrivedAtGarment_in;
		struct GarmentGripped_t : public EventBuffer
		{
			THREAD_SAFE_ONLY(std::mutex _mutex;)
			std::tuple<> _args;
			void* _sender = nullptr;
			void* getSender() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				return _sender;
			}
			
			void reset() override
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = nullptr;
			}
			
			void trigger(void* sender, std::tuple<> args)
			{
				THREAD_SAFE_ONLY(std::lock_guard<std::mutex> lock{_mutex};)
				_sender = sender;
			}
		} garmentGripped_in;
	};
	
	Channels channels{*this};
	
	Franka* platform;
	SDressingControl_StateMachine<CDressingControl> sDressingControl{*platform, *this, &sDressingControl};
	MovementControl_StateMachine<CDressingControl> movementControl{*platform, *this, &movementControl};
	EmergencyStopControl_StateMachine<CDressingControl> emergencyStopControl{*platform, *this, &emergencyStopControl};
	EnvironmentalMonitor_StateMachine<CDressingControl> environmentalMonitor{*platform, *this, &environmentalMonitor};
};

#endif
