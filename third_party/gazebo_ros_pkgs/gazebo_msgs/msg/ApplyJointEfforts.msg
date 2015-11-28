# set urdf joint effort
string[] name               # joint to apply wrench (linear force and torque)
float64[] effort            # effort to apply
time start_time             # optional wrench application start time (seconds)
                            # if start_time < current time, start as soon as possible
duration duration           # optional duration of wrench application time (seconds)
                            # if duration < 0, apply wrench continuously without end
                            # if duration = 0, do nothing
                            # if duration < step size, assume step size and
                            #               display warning in status_message
