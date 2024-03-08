import math

_64_s_en1, _64_s_I_error = 0, 0
_32_s_en1, _32_s_I_error = 0, 0
_ou_s_en1, _ou_s_i_error = 0, 0
_64_d_en2, _64_d_en1, _64_d_Mv1 = 0, 0, 0
_32_d_en2, _32_d_en1, _32_d_Mv1 = 0, 0, 0
_ou_d_error1, _ou_d_error2, _ou_d_Mv1 = 0, 0, 0

def fcn_plant(m_t):
    return math.pi * math.sin(2 * math.pi * m_t)

def fcn_sensor(m_yp):
    return m_yp

def fcn_adc(SenInRange, m_AdcR, m_ys):
    return int((m_AdcR * m_ys) / SenInRange)

def fcn_s_64(m_SenInRange, m_AdcR, m_SamT, m_AdcOut, i_D_ref, m_kp, m_ki, m_kd, m_PWM_DuraionT, m_PWM_IOclkT):
    global _64_s_en1, _64_s_I_error
    _64en0 = (m_SenInRange * i_D_ref) / m_AdcR - m_SenInRange * m_AdcOut / m_AdcR
    _64pError = _64en0
    _64_s_I_error += _64en0
    _64dError = _64en0 - _64_s_en1
    _64_s_Mv = m_kp * _64pError + m_ki * m_SamT * _64_s_I_error + m_kd * _64dError / m_SamT
    _64_s_en1 = _64en0
    return _64_s_Mv



def fcn_d_64(m_SenInRange, m_AdcR, m_SamT, m_AdcOut, i_D_ref, m_kp, m_ki, m_kd, m_PWM_DuraionT, m_PWM_IOclkT):
    global _64_d_en2, _64_d_en1, _64_d_Mv1
    _64r = (m_SenInRange * i_D_ref) / m_AdcR
    _64yrn = m_SenInRange * m_AdcOut / m_AdcR
    _64_d_en0 = _64r - _64yrn
    _64b0 = m_kp + (m_SamT * m_ki) / 2 + m_kd / m_SamT
    _64b1 = (m_ki * m_SamT) / 2 - m_kp - (2 * m_kd) / m_SamT
    _64b2 = m_kd / m_SamT
    _64_d_Mv0 = _64_d_Mv1 + _64b0 * _64_d_en0 + _64b1 * _64_d_en1 + _64b2 * _64_d_en2
    _64_d_en2, _64_d_en1, _64_d_Mv1 = _64_d_en1, _64_d_en0, _64_d_Mv0
    return _64_d_Mv0


def fcn_Initial(i_D_ref):
    global _64_s_en1, _64_s_I_error, _32_s_en1, _32_s_I_error, _ou_s_en1, _ou_s_i_error
    global _64_d_en2, _64_d_en1, _64_d_Mv1, _32_d_en2, _32_d_en1, _32_d_Mv1, _ou_d_error1, _ou_d_error2, _ou_d_Mv1
    _64_s_en1, _64_s_I_error = 0, 0
    _32_s_en1, _32_s_I_error = 0, 0
    _ou_s_en1, _ou_s_i_error = 0, 0
    _64_d_en2, _64_d_en1, _64_d_Mv1 = 0, 0, 0
    _32_d_en2, _32_d_en1, _32_d_Mv1 = 0, 0, 0
    _ou_d_error1, _ou_d_error2, _ou_d_Mv1 = 0, 0, 0

def main():
    AdcR = 1024  # adc 10bit
    SamT = 0.001953125
    SenInRange = 2 * math.pi




    f_kp, f_ki, f_kd = 1.17, 0.02, -0.21;
    i_kp, i_ki, i_kd = 1.17, 0.02, -0.21;
    PWM_DuraionT, PWM_IOclkT = 0.0001, 0.0001
    i_D_ref = 0

    fcn_Initial(i_D_ref)

    print("\nS64 = [", end=" ")
    for t1 in range(0, 512):
        t1 = t1 * SamT
        yp = fcn_plant(t1)
        ys = fcn_sensor(yp)
        AdcOut = fcn_adc(SenInRange, AdcR, ys)
        u64 = fcn_s_64(SenInRange, AdcR, SamT, AdcOut, i_D_ref, f_kp, f_ki, f_kd, PWM_DuraionT, PWM_IOclkT)
        print(u64, end=", ")
    print("]")



    print("\nD64 = [", end=" ")
    for t4 in range(0, 512):
        t4 = t4 * SamT
        yp = fcn_plant(t4)
        ys = fcn_sensor(yp)
        AdcOut = fcn_adc(SenInRange, AdcR, ys)
        u64 = fcn_d_64(SenInRange, AdcR, SamT, AdcOut, i_D_ref, f_kp, f_ki, f_kd, PWM_DuraionT, PWM_IOclkT)
        print(u64, end=", ")
    print("]")



if __name__ == "__main__":
    main()
