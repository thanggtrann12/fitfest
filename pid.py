class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.error = 0
        self.integral = 0
        self.derivative = 0
        self.prev_error = 0
        self.output = 0

    def update(self, process_variable):
        # Calculate the error between the setpoint and the process variable
        self.error = self.setpoint - process_variable

        # Update the integral term of the PID controller
        self.integral += self.error

        # Limit the integral term to prevent windup
        if self.integral > 100:
            self.integral = 100
        elif self.integral < -100:
            self.integral = -100

        # Calculate the derivative term of the PID controller
        self.derivative = self.error - self.prev_error

        # Update the output of the PID controller
        self.output = self.Kp*self.error + self.Ki * \
            self.integral + self.Kd*self.derivative

        # Update the previous error for the next iteration
        self.prev_error = self.error

        # Return the output
        return self.output
