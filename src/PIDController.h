#ifndef PIDCONTROLLER_H
	#define PIDCONTROLLER_H

	class PIDController {
	private:

	public:
		float Kp, Ki, Kd, dt, _e_prev, _i;
		PIDController(float Kp_ = 1, float Ki_ = 1, float Kd_ = 1);
		float calc(float x, float xd, float dt);
	};
#endif // !PIDCONTROLLER_H
