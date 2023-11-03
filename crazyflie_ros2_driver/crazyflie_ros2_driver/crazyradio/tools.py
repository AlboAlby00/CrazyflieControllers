import math


def convert_thrust(in_: float):
    """
    - converts scalar from 2.5 to 10 to range of 10001 to 60000
    - returns 0 if in_ is 0
    - returns int
    """
    thrust_in = in_ if in_ <= 10 else 10
    thrust_in = thrust_in - 2.5 if thrust_in > 2.5 else 0
    thrust_in = 10001 + thrust_in * ((40500 - 10001) / (7.5)) if thrust_in != 0 else 0
    return int(thrust_in)

def radian_to_degree(in_: float):
    return float((in_ / (math.pi)) * 360)