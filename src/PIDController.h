#ifndef PIDCONTROLLER_H
	#define PIDCONTROLLER_H

	class PIDController {
	private:

	public:
		double Kp, Ki, Kd, dt, _e_prev, _i;
		PIDController(double Kp_ = 1, double Ki_ = 1, double Kd_ = 1);
		double calc(double x, double xd, double dt);
	};
#endif // !PIDCONTROLLER_H
