class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # 비례 상수
        self.ki = ki  # 적분 상수
        self.kd = kd  # 미분 상수
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        """ PID 계산을 업데이트하고 제어 값을 반환합니다.
        :param error: 현재 오차 (선의 중심과 차량의 중심 사이)
        :param dt: 이전 업데이트 이후의 시간 간격
        :return: 제어 출력
        """
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output
