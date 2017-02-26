fgfs --fg-aircraft=$(pwd)/FlightGear/Aircraft/ \
--generic=socket,out,50,127.0.0.1,50001,udp,skyscanner \
--generic=socket,in,50,127.0.0.1,51001,udp,skyscanner \
--generic=socket,out,1,127.0.0.1,50002,udp,skyscanner_wind \
--generic=socket,in,1,127.0.0.1,51002,udp,skyscanner_wind \
--telnet=a,a,1000,a,5401,a \
--httpd=5480 \
--control=keyboard \
--aircraft=malolo1 \
--timeofday=noon \
--altitude=5300 \
--heading=0 \
--roll=0 \
--pitch=0 \
--vc=30 \
--roc=0 \
--enable-terrasync \
--lat=43.56195 \
--lon=1.47751 \
--wind=0@0 \
--turbulence=0.0 \
--disable-real-weather-fetch

# vc (airspeed): 30knots ~ 15m/s
# altitude: 3300ft ~ 1000m above sea level
# roc (rate of climb): 200 feet per minute[fpm] ~ 1 m/s
# --lat=43.56195 --lon=1.47751 -> Toulouse LAAS
