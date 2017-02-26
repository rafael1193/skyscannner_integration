bool circle                     # true if circle, false if line
geometry_msgs/Point origin      # center in circles, origin in lines
geometry_msgs/Point destination # unused in circles, destination in lines
float64 radius                  # radius in circles, unused in lines
float64 duration                # expected duration
float64 time_limit              # time after it the curve should be discarded
